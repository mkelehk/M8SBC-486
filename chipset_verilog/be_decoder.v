//=============================================================================
// 模块名 : be_decoder
// 功能   : 从 486 的字节使能信号 BE0~BE3 反推出 A1 / A0_BLE / BHE
//          参考 486DX 数据手册 (Oct92) 第 102 页的字节使能编码表
// 项目   : Hamster 1 chipset (M8SBC-486 REV 1.0)
// 原文件 : be_decoder.vhd
//
// 说明   : 486 在突发/多字节周期里用 BE3..BE0 表示哪几个字节有效，
//          而存储器件仍然需要传统的 A1 / A0(字节低位) / BHE 信号。
//          BE 为高有效，输出 A0_BLE / BHE 为高有效。
//=============================================================================
`timescale 1ns / 1ps

module be_decoder (
    input  wire BE0,        // 字节使能 0 (最低字节)
    input  wire BE1,        // 字节使能 1
    input  wire BE2,        // 字节使能 2
    input  wire BE3,        // 字节使能 3 (最高字节)

    output wire A1,         // 地址位 A1
    output wire A0_BLE,     // 字节低位地址 (低字节使能)
    output wire BHE         // 总线高字节使能 (高字节使能)
);

    // BE0 与 BE1 同时有效 => 访问的是 16 位字，A1 有效
    assign A1     = BE0 & BE1;

    // BE1 与 BE3 同时有效 => 高字节被选中
    assign BHE    = BE1 & BE3;

    // 三态：BE0/BE2 有效(低 16 位字的两个字节)，
    // 或者 BE0 有效但 BE1 无效(单独的奇地址字节)
    assign A0_BLE = (BE0 & BE2) | (BE0 & ~BE1);

endmodule
