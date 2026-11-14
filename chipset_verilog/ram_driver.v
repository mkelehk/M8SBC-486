//=============================================================================
// 模块名 : ram_driver
// 功能   : 4MB SRAM 驱动，存储器组织为 8 片 x 512KB (32 位数据宽度)
// 项目   : Hamster 1 chipset (M8SBC-486 REV 1.0)
// 原文件 : ram_driver.vhd
//
// 说明   : * CS0/CS1 为两个 2MB 存储体的片选(低有效)，由 A21 选择
//          * WE/OE 按字节给出(低有效)，WE 直接来自 CPU 的 BE 信号
//          * RDY 为高表示让 CPU 等待
//
// 读性能优化 (原设计 2026-01-13 增加):
//   如果连续两次都是对同一存储体的读操作，就保持 CS/OE 有效并跳过部分等待周
//   期，省掉"等待 ADS -> 再拉 CS/OE"的那一拍。
//   24 MHz FSB / 486DX2 / 70ns RAM 下实测 17.5 MB/s -> 26.8 MB/s。
//   (写操作的突发优化尚未实现)
//
// 时序余量提示 (24 MHz，1 拍 = 41.7 ns):
//   普通访问 RAM_WAITSTATES=1 -> 3 拍 125 ns，对 70 ns 器件余量充足；
//   突发读 RAM_BURST_WAITSTATES=0 -> 2 拍 83 ns，扣除地址缓冲与 SRAM 输出
//   有效时间后余量只剩约 13 ns，属于"实测可用但偏紧"。若换慢速 SRAM 或要
//   更大余量，把顶层的 RAM_BURST_WAITSTATES 改为 1 (3 拍 125 ns) 即可。
//=============================================================================
`timescale 1ns / 1ps

module ram_driver (
    input  wire       CLK,
    input  wire       RESET,                // 高有效
    input  wire [3:0] BE,                   // 字节使能，高有效
    input  wire       ADS,                  // 地址选通，低有效
    input  wire       CPU_RW,               // x86 定义：0 = 读, 1 = 写
    input  wire       RAMCS,                // 来自地址译码，低有效
    input  wire       ADDR21,               // 选择第 1/第 2 个 2MB 存储体

    output wire       CS0,                  // 低有效
    output wire       CS1,                  // 低有效
    output wire       RDY,                  // 1 = 等待, 0 = 就绪
    output reg  [3:0] WE,                   // 每字节写使能，低有效
    output reg  [3:0] OE,                   // 每字节输出使能，低有效

    input  wire [6:0] RAM_WAITSTATES,       // 普通访问等待周期
    input  wire [3:0] RAM_BURST_WAITSTATES  // 连续同体读的等待周期
);

    //-------------------------------------------------------------------------
    // 常量 / 状态
    //-------------------------------------------------------------------------
    localparam [6:0] NCOUNT  = 7'd16;  // 跨设备切换时额外增加的等待周期
    localparam       NACTIVE = 1'b0;   // 注意：本模块中该常量使额外等待分支失效

    localparam ST1_WAIT_FOR_ADS = 1'b0;
    localparam ST2_WAIT_STATE   = 1'b1;

    // drv_state 带显式初值并禁止自动重编码：
    // XST 会把 Verilog 的 case 状态机重新编码（isa_driver 上就发生过，
    // 3 位被扩成 6 位 one-hot，全 0 不是合法状态，报 Xst:2117）。
    // VHDL 侧是枚举类型，走 FSM 编译，上电状态由编码表决定、一定合法。
    // 这里让两侧都明确落在 ST1_WAIT_FOR_ADS 上，避免上电出现非法状态。
    (* signal_encoding = "user" *)
    reg        drv_state = ST1_WAIT_FOR_ADS;
    reg        drv_next_state;

    reg        RDY_I;                  // RDY 内部寄存器
    reg  [6:0] WS_COUNT     = 7'd0;    // 已等待周期数
    reg  [6:0] WS_TO_WAIT;             // 本次传输需要等待的周期数

    reg        LAST_CS      = 1'b1;    // 上一周期的 RAMCS
    reg        EXTRA_WS     = 1'b0;

    // 连续读(突发读)相关
    reg        LAST_CS0     = 1'b1;    // 上一周期使用的存储体
    reg        LAST_CS1     = 1'b1;
    reg        KEEP_READ    = 1'b0;    // 保持读期间 CS/OE 有效

    wire       d_cs0;                  // A21 译码出的存储体选择
    wire       d_cs1;

    integer    ram_waitstates_total;   // 本周期实际等待数(组合变量)
    reg        allow_drive;            // 输出译码用临时变量

    //=========================================================================
    // 存储体选择：A21=1 选第二个体(2-4MB)，否则选第一个体
    //=========================================================================
    assign d_cs0 = (ADDR21 == 1'b1) ? 1'b1 : 1'b0;
    assign d_cs1 = (ADDR21 == 1'b1) ? 1'b0 : 1'b1;

    assign CS0 = d_cs0;
    assign CS1 = d_cs1;

    //=========================================================================
    // 时序进程
    //=========================================================================
    always @(posedge CLK) begin
        if (RESET) begin
            drv_state  <= ST1_WAIT_FOR_ADS;
            WS_COUNT   <= 7'd0;
            EXTRA_WS   <= 1'b0;
            WS_TO_WAIT <= 7'd0;
            RDY_I      <= 1'b0;
            KEEP_READ  <= 1'b0;
            LAST_CS0   <= 1'b0;
            LAST_CS1   <= 1'b0;
        end
        else begin
            //-------------------------------------------------------------
            // 计算本次传输的等待周期数
            // NACTIVE = '0'，所以"跨设备追加 NCOUNT"的分支实际上不会成立，
            // 这里保留原有结构便于以后调整。
            //-------------------------------------------------------------
            if ((LAST_CS != RAMCS) && (NACTIVE == 1'b1)) begin
                ram_waitstates_total = RAM_WAITSTATES + NCOUNT;
            end
            else begin
                ram_waitstates_total = RAM_WAITSTATES;
            end

            // 切到其它设备或切换到写操作 => 结束"保持读"状态
            if ((RAMCS == 1'b1) || (CPU_RW == 1'b1)) begin
                KEEP_READ <= 1'b0;
                LAST_CS0  <= 1'b0;
                LAST_CS1  <= 1'b0;
            end

            if (drv_state == ST1_WAIT_FOR_ADS) begin
                if ((drv_next_state == ST2_WAIT_STATE) && (RAMCS == 1'b0)) begin
                    //-----------------------------------------------------
                    // 由 S1 跳到 S2 (ADS 到来)：确定等待周期
                    //-----------------------------------------------------
                    // 注意(与原始 VHDL 的差异)：
                    //   原 VHDL 在本进程内又声明了局部 VARIABLE d_cs0/d_cs1，
                    //   遮蔽了同名信号且从未赋值，仿真中恒为 'U'，
                    //   导致这个比较结果为假、快通道在仿真里从不启用。
                    //   这里按设计意图比较真实的存储体选择信号，
                    //   使"同存储体连续读"能真正走 RAM_BURST_WAITSTATES。
                    //   (d_cs0 与 d_cs1 互斥，但这里两个比较都不能省：
                    //    发生写周期或非 RAM 访问时 LAST_CS0/LAST_CS1 会被一起清 0，
                    //    只有两项同时相等才说明"上一次确实是一次同体 RAM 读"。)
                    if ((LAST_CS0 == d_cs0) && (LAST_CS1 == d_cs1) &&
                        (CPU_RW == 1'b0)) begin
                        // 与上一周期同体并且都是读 => 走突发读的快通道
                        KEEP_READ            <= 1'b1;
                        ram_waitstates_total  = RAM_BURST_WAITSTATES;
                    end

                    if (ram_waitstates_total > 0) begin
                        RDY_I <= 1'b1;      // 需要等待时立刻抬高 RDY
                    end
                    else begin
                        RDY_I <= 1'b0;
                    end

                    WS_TO_WAIT <= ram_waitstates_total;

                    if ((LAST_CS != RAMCS) && (NACTIVE == 1'b1)) begin
                        EXTRA_WS <= 1'b1;
                    end
                    else begin
                        EXTRA_WS <= 1'b0;
                    end

                    // 记录本次使用的存储体，供下一次突发判断
                    LAST_CS0 <= d_cs0;
                    LAST_CS1 <= d_cs1;
                end
                else begin
                    RDY_I <= 1'b0;
                end
            end
            else begin
                if (drv_next_state == ST1_WAIT_FOR_ADS) begin
                    RDY_I <= 1'b0;          // 周期结束，撤销等待
                end
            end

            drv_state <= drv_next_state;

            // S2 中累加等待计数
            if (drv_state == ST2_WAIT_STATE) begin
                WS_COUNT <= WS_COUNT + 7'd1;
            end
            else begin
                WS_COUNT <= 7'd0;
            end

            LAST_CS <= RAMCS;
        end
    end

    //=========================================================================
    // 组合逻辑：输出译码
    //   写：WE 跟随字节使能 BE，OE 关闭
    //   读：OE 全部打开(BE 在缓存行填充时不可靠，读周期一律忽略)
    //   非传输期间：若处于"保持读"则 OE 继续有效，以隐藏下一次访问的建立时间
    //=========================================================================
    always @(*) begin
        if (EXTRA_WS == 1'b1) begin
            allow_drive = (WS_COUNT >= NCOUNT);
        end
        else begin
            allow_drive = 1'b1;
        end

        if ((drv_state == ST2_WAIT_STATE) && (allow_drive == 1'b1)) begin
            if (CPU_RW == 1'b1) begin
                OE = 4'b1111;
                WE = BE;                // BE 在整个周期内有效
            end
            else begin
                WE = 4'b1111;
                OE = 4'b0000;           // 读周期忽略 BE，可修复 L1 缓存填充的问题
            end
        end
        else begin
            WE = 4'b1111;
            if (KEEP_READ == 1'b1) begin
                OE = 4'b0000;           // 保持读：不关闭 OE，省掉一个等待周期
            end
            else begin
                OE = 4'b1111;
            end
        end
    end

    //=========================================================================
    // 组合逻辑：次态译码
    //=========================================================================
    always @(*) begin
        drv_next_state = drv_state;     // 默认保持

        case (drv_state)
            ST1_WAIT_FOR_ADS: begin
                if ((ADS == 1'b0) && (RAMCS == 1'b0)) begin
                    drv_next_state = ST2_WAIT_STATE;
                end
            end

            ST2_WAIT_STATE: begin
                if ((ADS == 1'b1) && (WS_COUNT >= WS_TO_WAIT)) begin
                    drv_next_state = ST1_WAIT_FOR_ADS;
                end
            end

            default: begin
                drv_next_state = ST1_WAIT_FOR_ADS;
            end
        endcase
    end

    assign RDY = RDY_I;

endmodule
