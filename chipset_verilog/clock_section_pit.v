//=============================================================================
// 模块名 : clock_section_pit
// 功能   : PIT(8254)时钟分频，14.318 MHz -> 1.193 MHz
// 项目   : Hamster 1 chipset (M8SBC-486 REV 1.0)
// 原文件 : clock_section_pit.vhd
//
// 说明   : 直接调用 Xilinx 原语 CLKDLL 的 CLKDV 输出来做固定分频。
//          1.19318 MHz = 14.31818 MHz / 12，是 IBM PC 的 PIT 基准时钟。
//          注意 : CLKDV_DIVIDE 的合法值是 1.5/2.0/2.5/3.0/4.0/5.0/8.0/16.0，
//                 这里故意填 12.0，虽然不在文档范围内，但在该器件上可用，
//                 等效于 48 MHz/4 后再次分频。修改前请确认 DLL 仍能锁定。
//=============================================================================
`timescale 1ns / 1ps

module clock_section_pit (
    input  wire CLK_INPUT,   // 14.318 MHz 输入
    output wire CLK_OUT      // 1.193 MHz 输出
);

    wire CLKDV_raw;   // DLL 分频输出(未经全局缓冲)
    wire CLK0_raw;    // DLL 同相输出，用于反馈
    wire CLKFB;       // 反馈时钟
    wire LOCKED_int;  // DLL 锁定指示(本模块未使用)

    //-------------------------------------------------------------------------
    // DLL：只使用 CLKDV 分频输出
    //-------------------------------------------------------------------------
    CLKDLL #(
        .CLKDV_DIVIDE         (12.0),   // 文档外取值，见文件头说明
        .DUTY_CYCLE_CORRECTION("TRUE"),
        .STARTUP_WAIT         ("TRUE")
    ) CLKDLL_inst (
        .CLKIN  (CLK_INPUT),    // 来自输入缓冲/时钟引脚
        .CLKFB  (CLKFB),        // 反馈来自 BUFG
        .RST    (1'b0),         // 不使用复位
        .CLKDV  (CLKDV_raw),    // 分频后的时钟
        .CLK0   (CLK0_raw),     // 用于反馈
        .CLK90  (),             // 未使用
        .CLK180 (),             // 未使用
        .CLK270 (),             // 未使用
        .CLK2X  (),             // 未使用
        .LOCKED (LOCKED_int)
    );

    // DLL 反馈缓冲，保证 CLK0 与 CLKIN 同相
    BUFG BUFG_FB (
        .I(CLK0_raw),
        .O(CLKFB)
    );

    // 分频时钟走全局时钟网络
    BUFG BUFG_CLKDV (
        .I(CLKDV_raw),
        .O(CLK_OUT)
    );

endmodule
