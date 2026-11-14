//=============================================================================
// 模块名 : m8sbc_main
// 功能   : Hamster 1 chipset 顶层，把所有子模块连接起来
// 板卡   : M8SBC-486 REV 1.0
// 原文件 : m8sbc_main.vhd
//
// 主要组成:
//   clock_section      48 MHz -> CPU 时钟 (DIVIDE_BY = 2 => 24 MHz)
//   clock_section_pit  14.318 MHz -> 1.193 MHz PIT 时钟
//   address_decoder    存储器/IO 片选
//   ram_driver         4MB SRAM (32 位)
//   wr_rd_generator    板载设备读写选通 + 等待周期
//   isa_driver         ISA 总线周期
//   transceiver_driver 字节交换收发器使能
//   be_decoder         BE -> A1/A0/BHE
//   KBC_8042           键盘控制器
//   CMOS               实时钟 + NVRAM
//
// 注意 : 修改 clock_section 的 DIVIDE_BY 后，必须同步修改下面的等待周期常量。
//=============================================================================
`timescale 1ns / 1ps

module m8sbc_main (
    //---------- 时钟输入 ----------
    input  wire        CLK_IN_MAIN,      // 48 MHz 主时钟
    input  wire        CLK_IN_14_318,    // 14.318 MHz (PIT/ISA 基准)

    //---------- 时钟输出 ----------
    output wire        CLK_OUT_CPU,
    output wire        CLK_OUT_ISA,
    output wire        CLK_OUT_PIT,

    //---------- CPU 输入 ----------
    input  wire [23:2] CPU_IN_ADDR,      // CPU 地址 A23..A2
    input  wire        CPU_IN_ADDR_31,   // CPU 地址 A31
    input  wire        CPU_IN_WR,        // 0 = 读, 1 = 写
    input  wire        CPU_IN_ADS,       // 地址选通，低有效
    input  wire        CPU_IN_MIO,       // 0 = IO, 1 = 存储器
    input  wire        CPU_IN_DC,        // 数据/控制周期指示
    input  wire [3:0]  CPU_IN_BE,        // 字节使能

    //---------- CPU 输出 ----------
    output wire        CPU_OUT_RDY,      // 1 = 让 CPU 等待
    output wire        CPU_OUT_BS8,      // 低有效
    output wire        CPU_OUT_BS16,     // 低有效
    output wire        CPU_OUT_KEN,      // 低有效
    output wire        CPU_OUT_NMI,

    //---------- CPU 双向数据总线 ----------
    inout  wire [7:0]  CPU_DATA,

    //---------- 生成的地址线 ----------
    output wire        ADDR_A0,
    output wire        ADDR_A1,

    //---------- 复位 ----------
    input  wire        RESET_SYS_IN,     // 高有效
    output wire        RESET_REQ_OUT,    // 低有效

    //---------- 配置 ----------
    input  wire        RAM_CACHE_EN,
    input  wire        ROM_CACHE_EN,

    //---------- RAM ----------
    output wire        RAM_CS0,
    output wire        RAM_CS1,
    output wire [3:0]  RAM_WE_B,
    output wire [3:0]  RAM_OE_B,

    //---------- ROM ----------
    output wire        ROM_CS,

    //---------- 字节交换收发器 ----------
    output wire [3:0]  TR_8B,
    output wire        TR_16B_LOW,
    output wire        TR_16B_HIGH,

    //---------- 通用控制 ----------
    output wire        MEM_WR,
    output wire        MEM_RD,
    output wire        IO_WR,
    output wire        IO_RD,

    //---------- ISA ----------
    output wire        ISA_MEM_WR,
    output wire        ISA_MEM_RD,
    output wire        ISA_SBHE,
    input  wire        ISA_MEMCS16,
    input  wire        ISA_IOCS16,
    input  wire        ISA_IO_READY,

    //---------- PIC (20h-21h) ----------
    output wire        PIC_CS,
    output wire        PIC_INTA,

    //---------- PIT (40h-43h) ----------
    output wire        PIT_CS,
    output wire        PIT_LATCH_LE,

    //---------- 键盘 (60h) ----------
    inout  wire        PS2_CLK,
    inout  wire        PS2_DATA,
    output wire        PS2_INTERUPT,

    //---------- CMOS NVRAM 的 AVR 接口 ----------
    input  wire        AVR_CLK,
    inout  wire        AVR_IO
);

    //=========================================================================
    // 常量
    //=========================================================================
    // 前 2 字节为芯片组标识，后 2 字节为版本号
    localparam [31:0] FPGA_VER = 32'h48860005;

    // 低于 16 MHz 运行时需要反相输出时钟，这里 24 MHz 所以为 0
    localparam REVERSE_CLOCK = 1'b0;

    // FSB 分频开关 (与 VHDL 的 FSB_DIVIDE 一致)：
    //   2 = 24 MHz (默认), 3 = 16 MHz, 4 = 12 MHz
    localparam integer FSB_DIVIDE = 2;

    // 以下等待周期对应 DIVIDE_BY = 2 (24 MHz)
    // 若把 FSB_DIVIDE 改成其它值，等待周期需按注释里给的对应关系同步调整：
    //   16 MHz: RAM_WS=2, ROM_WS=3, IO_WS=19
    //   12 MHz: RAM_WS=1, ROM_WS=2, IO_WS=12
    localparam [6:0] RAM_WAITSTATES       = 7'd1;   // 与 VHDL 的 RAM_WAITSTATES 一致
    localparam [3:0] RAM_BURST_WAITSTATES = 4'd0;   // 连续读等待，与 VHDL 一致
    localparam [6:0] ROM_WAITSTATES         = 7'd2;

    localparam [6:0] ONBOARD_IO_WAITSTATES  = 7'd23;
    localparam [6:0] PIC_INT_ACK_WAITSTATES = 7'd23;

    localparam [3:0] ISA_CLK_LOW_CYCLES     = 4'd2;   // ISA_CLK 约 8 MHz
    localparam [3:0] ISA_CLK_HIGH_CYCLES    = 4'd1;

    //=========================================================================
    // 内部信号
    //=========================================================================
    wire        CLK_CPU;          // CPU / FPGA 主时钟
    wire        CLK_14M_BUF;      // 14.318 MHz 缓冲后
    wire        CLK_PIT;          // 1.193 MHz
    wire        CLK_ISA;          // ISA 时钟

    wire        O_BS8;            // 实际送 CPU 的宽度指示
    wire        O_BS16;
    wire        ADDRDEC_BS8;      // 地址译码给出的宽度
    wire        ADDRDEC_BS16;
    wire        ISA_BS8;          // ISA 驱动给出的宽度
    wire        ISA_BS16;

    wire        I_CS_RAM;

    wire        O_IO_RD;
    wire        O_IO_WR;

    wire        O_RDY_RAM;
    wire        O_RDY_WRRD;
    wire        O_RDY_ISA;

    wire        O_A0_BLE;
    wire        O_A1;
    wire        O_BHE;

    wire        I_CS_ROM;
    wire        I_CS_PIC;
    wire        I_CS_PIT;
    wire        I_CS_PS2;
    wire        I_CS_O61;
    wire        I_CS_ISA;
    wire        I_CS_CMOS;

    reg         S_EN;             // 板载读写发生器使能(低有效)
    reg         S_ISA_EN;         // ISA 驱动使能(低有效)
    reg  [6:0]  S_WAITSTATES;
    reg  [6:0]  S_WAITSTATES_ISA16;

    wire        MUX_MEM_WR;
    wire        MUX_MEM_RD;
    wire        MUX_ISA_IO_WR;
    wire        MUX_ISA_IO_RD;

    wire        IO_RD_P;
    wire        IO_WR_P;
    wire        ISA_MEM_WR_P;
    wire        ISA_MEM_RD_P;

    reg  [7:0]  O_CPU_DATA;
    reg         O_CPU_DATA_P_O;   // 本周期由 FPGA 驱动数据总线
    reg  [7:0]  O61_DATA_L = 8'h00;

    wire [7:0]  O_PS2_DATA_OUT;
    wire        O_PS2_INT;

    wire [7:0]  O_CMOS_DATA_OUT;

    wire        O_PS2_RESET_REQ;

    wire        IO_PS2_CLK_IN;
    wire        IO_PS2_CLK_OUT;
    wire        IO_PS2_DAT_IN;
    wire        IO_PS2_DAT_OUT;

    wire        I_INT_ACK;

    wire        EXTRA_BS8;
    wire        EXTRA_BS16;

    wire        CPU_O_KEN;

    wire        O_CPU_16BTR;

    reg  [3:0]  SEL_BUS;          // 组合译码用临时变量

    //=========================================================================
    // 时钟生成
    //=========================================================================
    clock_section #(
        .DIVIDE_BY (FSB_DIVIDE)          // 48 MHz / FSB_DIVIDE
    ) CLKGEN (
        .CLK_INPUT  (CLK_IN_MAIN),
        .CPU_CLK_OUT(CLK_CPU)
    );

    // 14.318 MHz 输入缓冲
    IBUFG CLKIN_IBUFG (
        .I(CLK_IN_14_318),
        .O(CLK_14M_BUF)
    );

    clock_section_pit CLKGEN_PIT (
        .CLK_INPUT (CLK_14M_BUF),
        .CLK_OUT   (CLK_PIT)
    );

    //=========================================================================
    // RAM 驱动
    //=========================================================================
    ram_driver RAMDRV (
        .CLK                  (CLK_CPU),
        .RESET                (RESET_SYS_IN),
        .BE                   (CPU_IN_BE),
        .ADS                  (CPU_IN_ADS),
        .CPU_RW               (CPU_IN_WR),
        .RAMCS                (I_CS_RAM),        // 来自地址译码
        .ADDR21               (CPU_IN_ADDR[21]),

        .CS0                  (RAM_CS0),
        .CS1                  (RAM_CS1),
        .RDY                  (O_RDY_RAM),
        .WE                   (RAM_WE_B),
        .OE                   (RAM_OE_B),

        .RAM_WAITSTATES       (RAM_WAITSTATES),
        .RAM_BURST_WAITSTATES (RAM_BURST_WAITSTATES)
    );

    //=========================================================================
    // 字节交换收发器
    //=========================================================================
    transceiver_driver TRANSCEIVERDRV (
        .BE           (CPU_IN_BE),
        .BS8          (O_BS8),
        .BS16         (O_BS16),

        .TR_8B        (TR_8B),
        .TR_16B_LOW   (TR_16B_LOW),
        .TR_16B_HIGH  (TR_16B_HIGH)
    );

    //=========================================================================
    // BE -> A1 / A0 / BHE
    //=========================================================================
    be_decoder BEDECODER (
        .BE0    (CPU_IN_BE[0]),
        .BE1    (CPU_IN_BE[1]),
        .BE2    (CPU_IN_BE[2]),
        .BE3    (CPU_IN_BE[3]),

        .A1     (O_A1),
        .A0_BLE (O_A0_BLE),
        .BHE    (O_BHE)
    );

    //=========================================================================
    // 地址译码
    //=========================================================================
    address_decoder ADRDECODER (
        .ADDR_IN       (CPU_IN_ADDR),
        .ADDR_31       (CPU_IN_ADDR_31),
        .ADDR_A0       (O_A0_BLE),
        .ADDR_A1       (O_A1),
        .CPU_MIO       (CPU_IN_MIO),
        .CPU_WR        (CPU_IN_WR),

        .RAM_CACHEABLE (RAM_CACHE_EN),
        .ROM_CACHEABLE (ROM_CACHE_EN),

        .INT_ACK       (I_INT_ACK),

        .RAM_CS        (I_CS_RAM),
        .ROM_CS        (I_CS_ROM),
        .PIC_CS        (I_CS_PIC),
        .PIT_CS        (I_CS_PIT),
        .PS2_CS        (I_CS_PS2),
        .O61_CS        (I_CS_O61),
        .CMOS_CS       (I_CS_CMOS),
        .ISA_CS        (I_CS_ISA),

        .OUT_KEN       (CPU_O_KEN),
        .OUT_BS16      (ADDRDEC_BS16),
        .OUT_BS8       (ADDRDEC_BS8)
    );

    //=========================================================================
    // 板载设备读写选通 + 等待周期
    //=========================================================================
    wr_rd_generator WRRDGEN (
        .CLK           (CLK_CPU),
        .RESET         (RESET_SYS_IN),
        .ADS           (CPU_IN_ADS),
        .RW            (CPU_IN_WR),
        .MIO           (CPU_IN_MIO),
        .EN_WRRD       (S_EN),
        .WAITSTATE_CNT (S_WAITSTATES),

        .RDY           (O_RDY_WRRD),
        .MEM_WR        (MUX_MEM_WR),
        .MEM_RD        (MUX_MEM_RD),
        .IO_WR         (O_IO_WR),
        .IO_RD         (O_IO_RD)
    );

    //=========================================================================
    // ISA 驱动
    //=========================================================================
    isa_driver ISA_DRV (
        .CLK                (CLK_CPU),
        .RESET              (RESET_SYS_IN),
        .ADS                (CPU_IN_ADS),
        .RW                 (CPU_IN_WR),
        .MIO                (CPU_IN_MIO),
        .EN_ISA             (S_ISA_EN),

        .ISA_CLK_HIGH_CYCLES(ISA_CLK_HIGH_CYCLES),
        .ISA_CLK_LOW_CYCLES (ISA_CLK_LOW_CYCLES),
        .WAITSTATE_16C      (S_WAITSTATES_ISA16[3:0]),
        .WAITSTATE_END      (S_WAITSTATES),

        .ISA_MEMCS16        (ISA_MEMCS16),
        .ISA_IOCS16         (ISA_IOCS16),
        .ISA_IO_READY       (ISA_IO_READY),

        .ISA_RDY            (O_RDY_ISA),
        .ISA_MEM_WR         (ISA_MEM_WR_P),
        .ISA_MEM_RD         (ISA_MEM_RD_P),
        .ISA_IO_WR          (MUX_ISA_IO_WR),
        .ISA_IO_RD          (MUX_ISA_IO_RD),
        .ISA_CLK            (CLK_ISA),

        .BS8_O              (ISA_BS8),
        .BS16_O             (ISA_BS16),

        .CPU_16BTR          (O_CPU_16BTR),
        .ISA_SBHE           (ISA_SBHE)
    );

    //=========================================================================
    // 键盘控制器
    //=========================================================================
    KBC_8042 KB8042CTRL (
        .CLK         (CLK_CPU),
        .CLK_1_1931M (CLK_PIT),
        .RST_n       (~RESET_SYS_IN),

        .CS_n        (I_CS_PS2),
        .RD_n        (O_IO_RD),
        .WR_n        (O_IO_WR),
        .A2          (CPU_IN_ADDR[2]),
        .DATA_IN     (CPU_DATA),
        .DATA_OUT    (O_PS2_DATA_OUT),

        .PS2_CLK_IN  (IO_PS2_CLK_IN),
        .PS2_CLK_OUT (IO_PS2_CLK_OUT),
        .PS2_DAT_IN  (IO_PS2_DAT_IN),
        .PS2_DAT_OUT (IO_PS2_DAT_OUT),

        .IRQ1        (O_PS2_INT),
        .A20_GATE    (),                        // 本板未使用
        .CPU_RESET_n (O_PS2_RESET_REQ)          // 低有效，与复位请求极性一致
    );

    //=========================================================================
    // CMOS RTC / NVRAM
    //=========================================================================
    CMOS CMOS_INST (
        .CLK_IN   (CLK_CPU),
        .DATA_IN  (CPU_DATA),
        .DATA_OUT (O_CMOS_DATA_OUT),
        .CMOS_CS  (I_CS_CMOS),
        .WR       (O_IO_WR),
        .RD       (O_IO_RD),
        .A0       (O_A0_BLE),

        .CLK_PIT  (CLK_PIT),

        .AVR_CLK  (AVR_CLK),
        .AVR_IO   (AVR_IO),

        .FPGA_VER (FPGA_VER),
        .RESET    (RESET_SYS_IN)
    );

    //=========================================================================
    // 组合逻辑：中断响应检测、总线归属
    //=========================================================================
    // 486 在中断响应周期会同时拉低 DC 与 MIO
    assign I_INT_ACK = ((CPU_IN_DC == 1'b0) && (CPU_IN_MIO == 1'b0)) ? 1'b0 : 1'b1;

    // BHE 兼作"CPU 做 16 位传输"的指示
    assign O_CPU_16BTR = O_BHE;

    // ISA 有效时把读写选通交给 ISA 驱动，否则交给板载发生器
    assign IO_RD_P = (I_CS_ISA == 1'b1) ? O_IO_RD : MUX_ISA_IO_RD;
    assign IO_WR_P = (I_CS_ISA == 1'b1) ? O_IO_WR : MUX_ISA_IO_WR;

    assign IO_RD      = IO_RD_P;
    assign IO_WR      = IO_WR_P;
    assign MEM_WR     = MUX_MEM_WR;
    assign MEM_RD     = MUX_MEM_RD;
    assign ISA_MEM_WR = ISA_MEM_WR_P;
    assign ISA_MEM_RD = ISA_MEM_RD_P;

    // 总线宽度来源选择
    assign EXTRA_BS8  = (I_CS_ISA == 1'b1) ? ADDRDEC_BS8  : ISA_BS8;
    assign EXTRA_BS16 = (I_CS_ISA == 1'b1) ? ADDRDEC_BS16 : ISA_BS16;

    // 中断响应周期固定按 8 位处理
    assign O_BS8  = (I_INT_ACK == 1'b0) ? 1'b0 : EXTRA_BS8;
    assign O_BS16 = (I_INT_ACK == 1'b0) ? 1'b1 : EXTRA_BS16;

    //=========================================================================
    // 组合逻辑：选择哪个设备响应本次周期，并给出对应等待周期
    //
    // INTA 周期说明:
    //   CPU 同时拉低 DC/MIO/WR 表示中断响应，此时 A2-A31 为 0，
    //   CPU 会连续做两次 IO 读；PIC 忽略片选，因此直接把两次 IO_RD
    //   当作两根 INTA 脉冲送出去。
    //=========================================================================
    always @(*) begin
        SEL_BUS = {I_CS_ROM,
                   (I_CS_PIC & I_CS_PIT & I_CS_O61 & I_CS_PS2 & I_CS_CMOS),
                   I_CS_ISA,
                   I_INT_ACK};

        // 默认：不选中任何设备
        S_EN               = 1'b1;
        S_ISA_EN           = 1'b1;
        S_WAITSTATES_ISA16 = 7'd0;
        S_WAITSTATES       = 7'd0;

        case (SEL_BUS)
            4'b0111: begin
                // ROM 访问
                S_EN         = 1'b0;
                S_WAITSTATES = ROM_WAITSTATES;
            end

            4'b1011: begin
                // 板载 IO 设备 (PIT / 键盘 / CMOS / 61h 等)
                S_EN         = 1'b0;
                S_WAITSTATES = ONBOARD_IO_WAITSTATES;
            end

            4'b1101: begin
                // ISA 访问
                S_EN     = 1'b1;      // 板载发生器不参与
                S_ISA_EN = 1'b0;      // 启动 ISA 驱动
            end

            4'b1110: begin
                // PIC 中断响应 (此时 ISA_CS 为低，所以单独判断)
                S_EN         = 1'b0;
                S_WAITSTATES = PIC_INT_ACK_WAITSTATES;
            end

            default: begin
                S_WAITSTATES       = 7'd0;
                S_EN               = 1'b1;
                S_ISA_EN           = 1'b1;
                S_WAITSTATES_ISA16 = 7'd0;
            end
        endcase
    end

    assign PIC_CS = I_CS_PIC;
    assign PIT_CS = I_CS_PIT;

    //=========================================================================
    // 组合逻辑：FPGA 向 CPU 数据总线输出读数据
    // 只有读周期的 IO 读，并且命中的是板载可读端口时才驱动总线。
    //=========================================================================
    always @(*) begin
        O_CPU_DATA    = 8'hzz;
        O_CPU_DATA_P_O = 1'b0;

        if (CPU_IN_WR == 1'b0) begin        // 只允许在读周期驱动
            if (O_IO_RD == 1'b0) begin
                if (I_CS_PS2 == 1'b0) begin
                    O_CPU_DATA     = O_PS2_DATA_OUT;
                    O_CPU_DATA_P_O = 1'b1;
                end
                else if (I_CS_O61 == 1'b0) begin
                    O_CPU_DATA     = O61_DATA_L;
                    O_CPU_DATA_P_O = 1'b1;
                end
                else if (I_CS_CMOS == 1'b0) begin
                    O_CPU_DATA     = O_CMOS_DATA_OUT;
                    O_CPU_DATA_P_O = 1'b1;
                end
            end
        end
    end

    assign CPU_DATA = (O_CPU_DATA_P_O == 1'b1) ? O_CPU_DATA : 8'hzz;

    //=========================================================================
    // 端口 61h：外部是锁存器，本身不可读，
    // 这里在写 61h 时把数据也存一份，读 61h 时回送，模拟可读效果。
    //=========================================================================
    // 74LS573 锁存器低电平锁存、高电平直通，故 LE 取反
    assign PIT_LATCH_LE = (I_CS_O61 == 1'b0) ? (~O_IO_WR) : 1'b0;

    always @(negedge CLK_CPU or posedge RESET_SYS_IN) begin
        if (RESET_SYS_IN == 1'b1) begin
            O61_DATA_L <= 8'h00;
        end
        else if ((I_CS_O61 == 1'b0) && (O_IO_WR == 1'b0)) begin
            // 译码器在上升沿更新，故在下降沿采样数据
            O61_DATA_L <= CPU_DATA;
        end
    end

    //=========================================================================
    // PS/2 引脚处理：输出 0 时拉低，否则高阻(由外部上拉)
    // 读回的引脚值同时送给控制器；外部上拉保证空闲时为高。
    //=========================================================================
    assign PS2_CLK  = (IO_PS2_CLK_OUT == 1'b0) ? 1'b0 : 1'bz;
    assign IO_PS2_CLK_IN = PS2_CLK;

    assign PS2_DATA = (IO_PS2_DAT_OUT == 1'b0) ? 1'b0 : 1'bz;
    assign IO_PS2_DAT_IN = PS2_DATA;

    //=========================================================================
    // 其余输出
    //=========================================================================
    // 时钟输出直连：REVERSE_CLOCK 常量仅作说明保留，不参与逻辑
    // (16 MHz 以下的反相时钟方案已撤销，不增加时钟路径上的组合逻辑)
    assign CLK_OUT_CPU = CLK_CPU;
    assign CLK_OUT_PIT = CLK_PIT;
    assign CLK_OUT_ISA = CLK_ISA;

    assign CPU_OUT_BS8  = O_BS8;
    assign CPU_OUT_BS16 = O_BS16;

    assign ADDR_A0 = O_A0_BLE;
    assign ADDR_A1 = O_A1;

    assign PS2_INTERUPT = O_PS2_INT;

    assign ROM_CS = I_CS_ROM;

    // 三个来源中任意一个要求等待，CPU 就必须等待
    assign CPU_OUT_RDY = O_RDY_ISA | O_RDY_RAM | O_RDY_WRRD;

    assign RESET_REQ_OUT = O_PS2_RESET_REQ;
    assign CPU_OUT_NMI   = 1'b0;

    // 8259 的 INTA 等同于一次读操作，486 会保持两次读，因此直接用 IO_RD
    assign PIC_INTA = (I_INT_ACK == 1'b0) ? O_IO_RD : 1'b1;

    assign CPU_OUT_KEN = CPU_O_KEN;

endmodule
