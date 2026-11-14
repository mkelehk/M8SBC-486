//=============================================================================
// 模块名 : wr_rd_generator
// 功能   : Intel 总线风格的读写控制信号发生器，带等待周期(WAIT STATE)支持
// 项目   : Hamster 1 chipset (M8SBC-486 REV 1.0)
// 原文件 : wr_rd_generator.vhd
//
// 说明   : 状态机在 ADS 有效时启动传输，按 WAITSTATE_CNT 产生等待，
//          然后拉低对应的 MEM/IO 读或写选通。
//          所有输出均低有效；RDY 为高表示"让 CPU 等待"。
//          EN_WRRD 是低有效的使能(由地址译码决定何时响应)。
//=============================================================================
`timescale 1ns / 1ps

module wr_rd_generator (
    input  wire       CLK,
    input  wire       RESET,          // 高有效
    input  wire       ADS,            // 地址选通，低有效
    input  wire       RW,             // 0 = 读, 1 = 写 (x86 定义)
    input  wire       MIO,            // 0 = IO, 1 = 存储器
    input  wire       EN_WRRD,        // 低有效：本模块负责本次周期
    input  wire [6:0] WAITSTATE_CNT,  // 等待周期数 0..127

    output wire       RDY,            // 1 = 等待, 0 = 就绪
    output reg        MEM_WR,
    output reg        MEM_RD,
    output reg        IO_WR,
    output reg        IO_RD
);

    //-------------------------------------------------------------------------
    // 常量 / 状态编码
    //-------------------------------------------------------------------------
    localparam [6:0] NCOUNT  = 7'd2;   // 跨设备切换时额外增加的等待周期
    localparam       NACTIVE = 1'b1;   // LAST_CS 与 EN_WRRD 比较时的"有效"电平

    localparam ST1_WAIT_FOR_ADS = 1'b0;   // 等待 ADS，周期开始
    localparam ST2_WAIT_STATE   = 1'b1;   // 等待周期计数中

    //-------------------------------------------------------------------------
    // 内部信号
    //-------------------------------------------------------------------------
    reg  [6:0] SET_WAITSTATES;        // 本次周期实际使用的等待周期数
    reg  [6:0] new_waitstates;        // 本拍组合计算出的等待周期数
    reg        drv_state, drv_next_state;

    reg        RDY_I;                 // RDY 的内部寄存器
    reg  [6:0] WS_COUNT = 7'd0;       // 已等待的周期数

    reg        LAST_CS  = 1'b1;       // 上一周期的 EN_WRRD，用于检测设备切换
    reg        EXTRA_WS = 1'b0;       // 是否追加了 NCOUNT 个额外等待

    reg        rd;                    // 输出译码用临时变量
    reg        wr;
    reg        allow_drive;

    //=========================================================================
    // 时序进程：状态寄存器 + 等待计数
    //=========================================================================
    always @(posedge CLK) begin
        if (RESET) begin
            drv_state       <= ST1_WAIT_FOR_ADS;
            WS_COUNT        <= 7'd0;
            SET_WAITSTATES  <= 7'd0;
            RDY_I           <= 1'b0;    // 注意：RDY 为寄存器输出
            EXTRA_WS        <= 1'b0;
            LAST_CS         <= 1'b1;
        end
        else begin
            // 本拍组合计算等待周期：切换设备时额外加 NCOUNT
            // 注意: 不能用寄存器 SET_WAITSTATES 判断，它保存的是上一次传输的值；
            // 复位后第一次 ROM 取指会因此被误判为"零等待"，RDY 立刻释放，
            // CPU 在 ROM 数据建立前就采样，取到非法指令后停机
            if ((LAST_CS != EN_WRRD) && (NACTIVE == 1'b1)) begin
                // 上一周期访问的是别的设备，需要额外等待
                new_waitstates = WAITSTATE_CNT + NCOUNT;
            end
            else begin
                new_waitstates = WAITSTATE_CNT;
            end

            if (drv_state == ST1_WAIT_FOR_ADS) begin
                if (drv_next_state == ST2_WAIT_STATE) begin
                    // 由 S1 跳到 S2：本拍确定等待周期数
                    SET_WAITSTATES <= new_waitstates;

                    if ((LAST_CS != EN_WRRD) && (NACTIVE == 1'b1)) begin
                        EXTRA_WS <= 1'b1;
                    end
                    else begin
                        EXTRA_WS <= 1'b0;
                    end

                    if (new_waitstates > 7'd0) begin
                        RDY_I <= 1'b1;      // 立刻抬高 RDY 表示需要等待
                    end
                    else begin
                        RDY_I <= 1'b0;
                    end
                end
                else begin
                    RDY_I <= 1'b0;          // 空闲时 RDY 为低
                end
            end
            else begin
                // ST2_WAIT_STATE 期间：仅在退出到 ST1 时撤销 RDY，
                // 与 VHDL 一致（等待期间 RDY 一直保持高）
                if (drv_next_state == ST1_WAIT_FOR_ADS) begin
                    RDY_I <= 1'b0;          // 周期结束，撤销等待
                end
            end

            drv_state <= drv_next_state;

            // 处于 S2 时累加等待计数，否则清零
            if (drv_state == ST2_WAIT_STATE) begin
                WS_COUNT <= WS_COUNT + 7'd1;
            end
            else begin
                WS_COUNT <= 7'd0;
            end

            LAST_CS <= EN_WRRD;
        end
    end

    //=========================================================================
    // 组合逻辑：输出译码
    //  RDY 为高期间不驱动读写信号；等到允许驱动后再按 RW/MIO 输出。
    //=========================================================================
    always @(*) begin
        // 默认全部无效(高电平)，避免产生锁存器
        rd      = 1'b1;
        wr      = 1'b1;
        MEM_WR  = 1'b1;
        MEM_RD  = 1'b1;
        IO_WR   = 1'b1;
        IO_RD   = 1'b1;

        // 若追加了跨设备等待，则要等 NCOUNT 个周期之后才开始驱动
        if (EXTRA_WS == 1'b1) begin
            if (WS_COUNT >= NCOUNT) begin
                allow_drive = 1'b1;
            end
            else begin
                allow_drive = 1'b0;
            end
        end
        else begin
            allow_drive = 1'b1;
        end

        if ((drv_state == ST2_WAIT_STATE) && (allow_drive == 1'b1)) begin
            if (RW == 1'b1) begin
                wr = 1'b0;              // 写
            end
            else begin
                rd = 1'b0;              // 读
            end

            if (MIO == 1'b1) begin
                MEM_WR = wr;
                MEM_RD = rd;
            end
            else begin
                IO_WR  = wr;
                IO_RD  = rd;
            end
        end
        else begin
            MEM_WR = 1'b1;
            MEM_RD = 1'b1;
            IO_WR  = 1'b1;
            IO_RD  = 1'b1;
        end
    end

    //=========================================================================
    // 组合逻辑：次态译码
    //=========================================================================
    always @(*) begin
        drv_next_state = drv_state;     // 默认保持当前状态，避免锁存器

        case (drv_state)
            ST1_WAIT_FOR_ADS: begin
                // ADS 有效且本模块被选中 => 开始传输
                if ((ADS == 1'b0) && (EN_WRRD == 1'b0)) begin
                    drv_next_state = ST2_WAIT_STATE;
                end
            end

            ST2_WAIT_STATE: begin
                // 退出 ST2 的条件与 VHDL 一致：
                //   周期结束(ADS 已释放) 且 等待周期已满足，或使能被撤销
                if (((ADS == 1'b1) && (WS_COUNT >= SET_WAITSTATES)) ||
                    (EN_WRRD == 1'b1)) begin
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
