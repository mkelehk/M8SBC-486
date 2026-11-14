//=============================================================================
// 模块名 : isa_driver
// 功能   : 8/16 位 ISA 总线周期驱动
// 项目   : Hamster 1 chipset (M8SBC-486 REV 1.0)
// 原文件 : isa_driver.vhd (Revision 0.02 - 改为同步实现)
//
//-----------------------------------------------------------------------------
// 一、它在系统里的位置
//-----------------------------------------------------------------------------
//   顶层 m8sbc_main 里有两套"总线周期发生器":
//     * wr_rd_generator : 负责板载设备(ROM / PIT / PS2 / CMOS / 61h)的 MEM/IO 选通
//     * isa_driver      : 负责 ISA 插槽的 MEM/IO 选通，并且自己产生 ISA_CLK
//   两者在顶层按地址译码结果二选一:
//     I_CS_ISA=1 => IO_RD/IO_WR 直接取本模块之外的 wr_rd_generator 输出
//     I_CS_ISA=0 => 取本模块的 MUX_ISA_IO_WR / MUX_ISA_IO_RD
//   本模块的 ISA_RDY 与另外两个 RDY 相或后送 CPU 的 RDY 引脚。
//
//   本模块只做"时序"，不做数据: 数据通路是板上的 74LS245/573 收发器，
//   由 transceiver_driver 与 address_decoder 的 BS8/BS16 控制。
//
//-----------------------------------------------------------------------------
// 二、关键信号(来源 / 含义 / 有效电平 / 时钟域)
//-----------------------------------------------------------------------------
//   CLK   : 48MHz/2 = 24MHz 的 CPU 时钟(CLK_CPU)，本模块唯一时钟域。
//           所有触发器都在它的上升沿更新 —— 和 VHDL 版完全一致。
//   RESET : 高有效，由 RESET_SYS_IN 直接驱动；同步复位(写在时钟沿内)。
//   ADS   : 来自 CPU 的地址选通，低有效。低电平表示新总线周期开始。
//   RW    : 来自 CPU，x86 语义 0=读 1=写(与直觉相反，注意别弄反)。
//   MIO   : 0=IO 周期，1=存储器周期。
//   EN_ISA: 低有效，由 address_decoder 的 ISA_CS 驱动；
//           为高表示"本次周期不归 ISA 管"，状态机立即回 S1。
//   ISA_*CS16 / ISA_IO_READY : 来自 ISA 插槽的应答信号。
//           CS16 低有效(从设备声明自己是 16 位)，
//           CHRDY(ISA_IO_READY) 低表示从设备还要继续等待。
//   所有 ISA_MEM_WR/RD、ISA_IO_WR/RD 输出都是低有效；ISA_RDY 高=要求等待。
//
//-----------------------------------------------------------------------------
// 三、为什么要有 ISA_CLK 分频器
//-----------------------------------------------------------------------------
//   ISA 总线是 8 MHz 量级的异步总线，而 CPU 时钟是 24 MHz。
//   如果直接用 CPU 时钟去打 ISA 时序，周期长度会随 CPU 频率变化，
//   ISA 卡(以及 74LS 系列的外围时序)就对不上。
//   所以这里用 DIV_COUNT 把 CPU 时钟分频成 ISA_CLK:
//     高电平持续 ISA_CLK_HIGH_CYCLES 个 CPU 周期(当前=1)
//     低电平持续 ISA_CLK_LOW_CYCLES  个 CPU 周期(当前=2)
//   于是 ISA_CLK 周期 = 3 个 CPU 周期 = 125ns ≈ 8MHz。
//   分频器自己就是一个小状态机(ISA_CLK_STATE + DIV_COUNT)，独立于主状态机。
//   注意: 这个分频器不受 RESET 影响(与 VHDL 一致)，复位期间继续跑。
//
//-----------------------------------------------------------------------------
// 四、主状态机: 状态 / 迁移条件 / 每个状态承担的动作
//-----------------------------------------------------------------------------
//   S1 ST1_WAIT_FOR_ADS      : 空闲。等 ADS=0 且 EN_ISA=0 才进 S2。
//                              进入 S2 的那一拍同时清 WS_COUNT / ISA_DO_16B。
//   S2 ST2_TS_WAIT_FOR_RISE  : 等 ISA_CLK 的一个上升沿(等价条件写成
//                              "ISA_CLK_STATE=0 且低电平计满")。
//   S3 ST3_TS_WAIT_FOR_FALL  : 等一个下降沿，凑出完整的第一个 ISA 时钟周期。
//   S4 ST4_TC1_WAIT_FOR_RISE : TC1 阶段。这里本应发 BALE 脉冲，本板无 BALE 引脚，
//                              所以只在进入 S5 的那一拍采样从设备的 CS16。
//   S5 ST5_TC1_WAIT_FOR_FALL : TC1 的后半段。16 位从设备从这一状态就可以
//                              提前拉读写选通(比 8 位早半个 ISA 周期)。
//   S6 ST6_WAITSTATES_WAIT   : 等待周期计数。每来一个 ISA 时钟上升沿 WS_COUNT+1
//                              (上限 5)。退出条件:
//                                16 位: WS_COUNT>=1 且 CHRDY=1
//                                8  位: WS_COUNT>=4 且 CHRDY=1
//                              这就是"1 个起始周期 + 4 个等待周期"的 8 位标准周期。
//   任意状态只要 EN_ISA 变高(地址译码改判成别的设备)就立刻回 S1。
//
//-----------------------------------------------------------------------------
// 五、为什么状态寄存器要写编码属性和初值(重要，勿删)
//-----------------------------------------------------------------------------
//   VHDL 里 drv_state 是枚举类型，XST 会走 FSM 编译流程，用 johnson 编码 3 位，
//   st1_wait_for_ads 正好是 "000"，上电即空闲态。
//   Verilog 里如果只写 case 状态机，XST 会把它重新编码成 one-hot 的 6 位寄存器
//   (综合报告: "Using one-hot encoding for signal <drv_state>" +
//    "INFO:Xst:2117 ... may lead to design initialization problems")。
//   6 位 one-hot 的 "000000" 不是任何合法状态，上电后第一拍输出译码会得出
//   RDY_I=1(因为 drv_state != ST1)，把 CPU 多按住一拍；这正是
//   "VHDL 能起、Verilog 起不来"那类最典型的差异来源。
//   所以这里显式:
//     (* signal_encoding = "user" *) -> 禁止 XST 重新编码，保持 3 位二进制
//     = ST1_WAIT_FOR_ADS             -> 显式 INIT，上电即 000 空闲态
//   两者合起来，综合出来的就是和 VHDL 一样的 3 个触发器 + 上电空闲。
//
//   另外 WAITSTATE_16C / WAITSTATE_END 在原设计里就没用上，保留只为接口兼容。
//=============================================================================
`timescale 1ns / 1ps

module isa_driver (
    input  wire       CLK,
    input  wire       RESET,              // 高有效
    input  wire       ADS,                // 地址选通，低有效
    input  wire       RW,                 // 0 = 读, 1 = 写
    input  wire       MIO,                // 0 = IO, 1 = 存储器
    input  wire       EN_ISA,             // 低有效

    input  wire [3:0] ISA_CLK_HIGH_CYCLES,// ISA_CLK 高电平持续的 CPU 周期数
    input  wire [3:0] ISA_CLK_LOW_CYCLES, // ISA_CLK 低电平持续的 CPU 周期数
    input  wire [3:0] WAITSTATE_16C,      // (未使用)
    input  wire [6:0] WAITSTATE_END,      // (未使用)

    input  wire       ISA_MEMCS16,        // 从设备声明 16 位存储器周期，低有效
    input  wire       ISA_IOCS16,         // 从设备声明 16 位 IO 周期，低有效
    input  wire       ISA_IO_READY,       // 来自 ISA 的 CHRDY

    output wire       ISA_RDY,            // 1 = 等待, 0 = 就绪
    output reg        ISA_MEM_WR,
    output reg        ISA_MEM_RD,
    output reg        ISA_IO_WR,
    output reg        ISA_IO_RD,
    output wire       ISA_CLK,

    output reg        BS8_O,              // 8 位总线指示，低有效
    output reg        BS16_O,             // 16 位总线指示，低有效

    input  wire       CPU_16BTR,          // CPU 做 8 位传输时禁止 16 位转换
    output wire       ISA_SBHE
);

    //-------------------------------------------------------------------------
    // 状态编码
    //-------------------------------------------------------------------------
    localparam [2:0] ST1_WAIT_FOR_ADS       = 3'd0;  // 等 ADS
    localparam [2:0] ST2_TS_WAIT_FOR_RISE   = 3'd1;  // 等 ISA_CLK 上升沿
    localparam [2:0] ST3_TS_WAIT_FOR_FALL   = 3'd2;  // 等 ISA_CLK 下降沿
    localparam [2:0] ST4_TC1_WAIT_FOR_RISE  = 3'd3;  // TC1 等上升沿(应发 BALE)
    localparam [2:0] ST5_TC1_WAIT_FOR_FALL  = 3'd4;  // TC1 等下降沿
    localparam [2:0] ST6_WAITSTATES_WAIT    = 3'd5;  // 等待周期

    // drv_state 必须带初值 + 禁止 XST 重新编码(详细原因见文件头第五节)。
    //   (* signal_encoding = "user" *) 让 XST 保持这里的 3 位二进制编码，
    //   与 VHDL 枚举类型的 3 位(johnson, st1=000) 在"合法状态集合"上等价；
    //   初值保证上电第一拍就是 S1 空闲态，不会出现 one-hot 全 0 的非法态。
    (* signal_encoding = "user" *)
    reg [2:0] drv_state = ST1_WAIT_FOR_ADS;
    reg [2:0] drv_next_state;

    reg [3:0] DIV_COUNT    = 4'd0;   // 分频计数
    reg       ISA_CLK_STATE = 1'b0;  // ISA_CLK 当前电平

    reg       RDY_I;                 // RDY 内部寄存器(组合产生)
    reg       ISA_DO_16B   = 1'b0;   // 本次传输按 16 位进行

    reg [6:0] WS_COUNT     = 7'd0;   // 已等待的 ISA 周期数

    reg       rd;                    // 输出译码用临时变量
    reg       wr;

    //=========================================================================
    // ISA_CLK 分频：高电平持续 ISA_CLK_HIGH_CYCLES 个 CPU 周期，
    //              低电平持续 ISA_CLK_LOW_CYCLES 个 CPU 周期
    //=========================================================================
    always @(posedge CLK) begin
        if (ISA_CLK_STATE == 1'b1) begin
            if (DIV_COUNT >= (ISA_CLK_HIGH_CYCLES - 4'd1)) begin
                ISA_CLK_STATE <= 1'b0;
                DIV_COUNT     <= 4'd0;
            end
            else begin
                DIV_COUNT <= DIV_COUNT + 4'd1;
            end
        end
        else begin
            if (DIV_COUNT >= (ISA_CLK_LOW_CYCLES - 4'd1)) begin
                ISA_CLK_STATE <= 1'b1;
                DIV_COUNT     <= 4'd0;
            end
            else begin
                DIV_COUNT <= DIV_COUNT + 4'd1;
            end
        end
    end

    assign ISA_CLK = ISA_CLK_STATE;

    //=========================================================================
    // 时序进程
    //=========================================================================
    always @(posedge CLK) begin
        if (RESET) begin
            WS_COUNT  <= 7'd0;
            ISA_DO_16B <= 1'b0;
        end
        else begin
            // 新周期开始：清计数与宽度标志
            if (drv_state == ST1_WAIT_FOR_ADS) begin
                WS_COUNT   <= 7'd0;
                ISA_DO_16B <= 1'b0;
            end

            // TC1 阶段采样从设备的 CS16 信号
            if ((drv_state == ST4_TC1_WAIT_FOR_RISE) &&
                (drv_next_state == ST5_TC1_WAIT_FOR_FALL)) begin
                if (MIO == 1'b1) begin
                    if (ISA_MEMCS16 == 1'b0) begin
                        ISA_DO_16B <= 1'b1;
                    end
                end
                else begin
                    if (ISA_IOCS16 == 1'b0) begin
                        ISA_DO_16B <= 1'b1;
                    end
                end
            end

            // 等待阶段：每来一个 ISA 时钟周期计数一次，最多 5 个
            if (drv_state == ST6_WAITSTATES_WAIT) begin
                if ((ISA_CLK_STATE == 1'b0) &&
                    (DIV_COUNT >= (ISA_CLK_LOW_CYCLES - 4'd1))) begin
                    if (WS_COUNT < 7'd5) begin
                        WS_COUNT <= WS_COUNT + 7'd1;
                    end
                end
            end

            drv_state <= drv_next_state;
        end
    end

    //=========================================================================
    // 组合逻辑：输出译码
    //  BS8/BS16 必须在第一个 RDY 有效之前就建立，所以进入 TC1 后就开始输出。
    //=========================================================================
    always @(*) begin
        // 默认值
        BS8_O      = 1'b1;
        BS16_O     = 1'b1;
        ISA_MEM_WR = 1'b1;
        ISA_MEM_RD = 1'b1;
        ISA_IO_WR  = 1'b1;
        ISA_IO_RD  = 1'b1;

        // 【关键修正】rd/wr 必须先给默认值。
        // 原写法只在 if 里给其中一个赋值（写时给 wr、读时给 rd），另一个则保留
        // 上一拍的值，综合器会为它推断出锁存器（Xst:737）。上电 INIT=0，
        // 于是"读周期里写选通也是 0""写周期里读选通也是 0"，读写同时有效，
        // ISA 总线直接冲突。
        // VHDL 原件在 OUTPUT_DECODE 里先 WR := '1'; RD := '1'; 再改其中一个，
        // 每次求值都会把非活动的那个重新置 1，这里补上同样的初值才等价。
        wr = 1'b1;
        rd = 1'b1;

        // 传输期间保持 CPU 等待
        RDY_I = (drv_state != ST1_WAIT_FOR_ADS) ? 1'b1 : 1'b0;

        // 进入 TC1 时应输出 BALE 脉冲，本设计没有 BALE 引脚，故留空

        // 在 TC1 下降沿和等待阶段输出总线宽度
        if ((drv_state == ST5_TC1_WAIT_FOR_FALL) ||
            (drv_state == ST6_WAITSTATES_WAIT)) begin
            if (ISA_DO_16B == 1'b1) begin
                BS16_O = 1'b0;
            end
            else begin
                BS8_O = 1'b0;
            end
        end

        // 16 位从设备可以提前一个状态拉读写选通
        if ((drv_state == ST6_WAITSTATES_WAIT) ||
            ((drv_state == ST5_TC1_WAIT_FOR_FALL) && (ISA_DO_16B == 1'b1))) begin
            if (RW == 1'b1) begin
                wr = 1'b0;               // 写
            end
            else begin
                rd = 1'b0;               // 读
            end

            if (MIO == 1'b1) begin
                ISA_MEM_WR = wr;
                ISA_MEM_RD = rd;
            end
            else begin
                ISA_IO_WR  = wr;
                ISA_IO_RD  = rd;
            end
        end
    end

    //=========================================================================
    // 组合逻辑：次态译码
    // 所有等待 ISA 时钟的地方都用 "ISA_CLK_STATE=0 且低电平计满" 判断上升沿，
    // 用 "ISA_CLK_STATE=0 且低电平计满" 之后的下一状态判断下降沿。
    //=========================================================================
    always @(*) begin
        drv_next_state = drv_state;       // 默认保持

        case (drv_state)
            ST1_WAIT_FOR_ADS: begin
                if ((ADS == 1'b0) && (EN_ISA == 1'b0)) begin
                    drv_next_state = ST2_TS_WAIT_FOR_RISE;
                end
            end

            ST2_TS_WAIT_FOR_RISE: begin
                if (EN_ISA == 1'b1) begin
                    drv_next_state = ST1_WAIT_FOR_ADS;  // ISA 片选被撤销
                end
                else if ((ISA_CLK_STATE == 1'b0) &&
                         (DIV_COUNT >= (ISA_CLK_LOW_CYCLES - 4'd1))) begin
                    drv_next_state = ST3_TS_WAIT_FOR_FALL;
                end
            end

            ST3_TS_WAIT_FOR_FALL: begin
                if (EN_ISA == 1'b1) begin
                    drv_next_state = ST1_WAIT_FOR_ADS;
                end
                else if ((ISA_CLK_STATE == 1'b0) &&
                         (DIV_COUNT >= (ISA_CLK_LOW_CYCLES - 4'd1))) begin
                    drv_next_state = ST4_TC1_WAIT_FOR_RISE;
                end
            end

            ST4_TC1_WAIT_FOR_RISE: begin
                if (EN_ISA == 1'b1) begin
                    drv_next_state = ST1_WAIT_FOR_ADS;
                end
                else if ((ISA_CLK_STATE == 1'b0) &&
                         (DIV_COUNT >= (ISA_CLK_LOW_CYCLES - 4'd1))) begin
                    drv_next_state = ST5_TC1_WAIT_FOR_FALL;
                end
            end

            ST5_TC1_WAIT_FOR_FALL: begin
                if (EN_ISA == 1'b1) begin
                    drv_next_state = ST1_WAIT_FOR_ADS;
                end
                else if ((ISA_CLK_STATE == 1'b0) &&
                         (DIV_COUNT >= (ISA_CLK_LOW_CYCLES - 4'd1))) begin
                    drv_next_state = ST6_WAITSTATES_WAIT;
                end
            end

            ST6_WAITSTATES_WAIT: begin
                if (EN_ISA == 1'b1) begin
                    drv_next_state = ST1_WAIT_FOR_ADS;
                end
                else if (ISA_DO_16B == 1'b1) begin
                    // 16 位：1 个等待周期
                    if ((ISA_CLK_STATE == 1'b0) &&
                        (DIV_COUNT >= (ISA_CLK_LOW_CYCLES - 4'd1)) &&
                        (WS_COUNT >= 7'd1) && (ISA_IO_READY == 1'b1)) begin
                        drv_next_state = ST1_WAIT_FOR_ADS;
                    end
                end
                else begin
                    // 8 位：4 个等待周期
                    if ((ISA_CLK_STATE == 1'b0) &&
                        (DIV_COUNT >= (ISA_CLK_LOW_CYCLES - 4'd1)) &&
                        (WS_COUNT >= 7'd4) && (ISA_IO_READY == 1'b1)) begin
                        drv_next_state = ST1_WAIT_FOR_ADS;
                    end
                end
            end

            default: begin
                drv_next_state = ST1_WAIT_FOR_ADS;
            end
        endcase
    end

    assign ISA_RDY  = RDY_I;
    assign ISA_SBHE = CPU_16BTR;   // 16 位传输时禁止系统高字节使能

endmodule
