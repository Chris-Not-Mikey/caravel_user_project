module RX_INTF #(
    parameter NOC_WID = 16
) (
    input                   clk,
    input                   rst,

    input [NOC_WID-1:0]     rx,
    input [7:0]             rx_bits,
    input                   rx_toggle,

    // NoC ports
    output wire             rx_req,
    output wire [1:0]       rx_d,
    input                   rx_ack
);

    localparam [2:0]
        IDLE = 3'b000,
        REQ_HI = 3'b001,
        DATA_HI = 3'b010,
        DATA_LO = 3'b011,
        WAIT_ACK = 3'b100;

    reg [2:0]           state;
    reg [NOC_WID-1:0]   rx_sr;
    reg [7:0]           bits_left;
    reg                 toggle_last;


    assign rx_req = (state != IDLE) ? 1 : 0;
    assign rx_d[0] = (state == DATA_HI && rx_sr[NOC_WID-1] == 0) ? 1 : 0;
    assign rx_d[1] = (state == DATA_HI && rx_sr[NOC_WID-1] == 1) ? 1 : 0;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            rx_sr <= 0;
            bits_left <= 0;
            toggle_last <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (rx_toggle != toggle_last) begin
                        state <= REQ_HI;
                        rx_sr <= rx;
                        bits_left <= rx_bits;
                        toggle_last <= rx_toggle;
                    end
                end
                REQ_HI: begin
                    if (rx_ack) begin
                        state <= DATA_HI;
                    end
                end
                DATA_HI: begin
                    if (~rx_ack) begin
                        state <= DATA_LO;
                    end
                end
                DATA_LO: begin
                    state <= WAIT_ACK;
                    rx_sr <= (rx_sr << 1);
                    bits_left <= bits_left - 1;
                end
                WAIT_ACK: begin
                    if (rx_ack) begin
                        if (bits_left == 0) begin
                            state <= IDLE;
                        end else begin
                            state <= DATA_HI;
                        end
                    end
                end
            endcase
        end
    end

endmodule


// module tb;

//     reg         clk, rst;
//     reg [15:0]  rx;
//     reg [7:0]   rx_bits;
//     reg         rx_toggle;

//     wire        rx_req;
//     wire [1:0]  rx_d;
//     wire        rx_ack;

//     RX_INTF i_RX_INTF (
//         .clk      (clk      ),
//         .rst      (rst      ),
//         .rx       (rx       ),
//         .rx_bits  (rx_bits  ),
//         .rx_toggle(rx_toggle),
//         .rx_req   (rx_req   ),
//         .rx_d     (rx_d     ),
//         .rx_ack   (rx_ack   )
//     );

//     always #5 clk = ~clk;

//     localparam [1:0]
//         IDLE = 2'b00,
//         ACK_HI = 2'b01,
//         ACK_LO = 2'b10;

//     reg [1:0] state;

//     wire rx_valid;

//     assign rx_valid = rx_d[0] | rx_d[1];
//     assign rx_ack = (state == ACK_HI) ? 1 : 0;

//     always @(posedge clk) begin
//         case (state)
//             IDLE: begin
//                 if (rx_req) begin
//                     state <= ACK_HI;
//                 end
//             end
//             ACK_HI: begin
//                 if (rx_valid) begin
//                     state <= ACK_LO;
//                 end else if (~rx_req) begin
//                     state <= IDLE;
//                 end
//             end
//             ACK_LO: begin
//                 if (~rx_valid) begin
//                     state <= ACK_HI;
//                 end
//             end
//         endcase
//     end

//     initial begin
//         $dumpfile("dump.vcd");
//         $dumpvars(0, tb);

//         clk = 0;
//         rst = 1;

//         state = IDLE;

//         rx = 0;
//         rx_bits = 0;
//         rx_toggle = 0;
//         #21;

//         rst = 0;
//         rx = 16'b1100_1010_0000_0000;
//         rx_bits = 8;
//         rx_toggle = ~rx_toggle;
//         #400;

//         #200;
//         $finish;
//     end
// endmodule
module TX_INTF #(
    parameter NOC_WID = 16
) (
    input                       clk,
    input                       rst,

    output reg [NOC_WID-1:0]    tx,

    // NoC ports
    input [2*NOC_WID-1:0]       tx_d,
    output wire                 tx_ack
);

    localparam [1:0]
        IDLE = 2'b00,
        DATA_LOAD = 2'b01,
        ACK_HI = 2'b10;

    reg [2:0]           state;

    wire [NOC_WID-1:0]  tx_valid_bitwise;
    wire                tx_valid;
    wire                tx_empty;

    genvar i;
    generate
        for (i = 0; i < NOC_WID; i = i+1) begin
            assign tx_valid_bitwise[i] = (tx_d[2*i] | tx_d[2*i+1]);
        end
    endgenerate

    assign tx_valid = &tx_valid_bitwise;
    assign tx_empty = ~(|tx_valid_bitwise);

    assign tx_ack = (state == ACK_HI) ? 1 : 0;

    integer j;
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            tx <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (tx_valid) begin
                        state <= DATA_LOAD;
                        for (j = 0; j < NOC_WID; j = j+1) begin
                            tx[j] <= tx_d[2*j+1];
                        end
                    end
                end
                DATA_LOAD: begin
                    state <= ACK_HI;
                end
                ACK_HI: begin
                    if (tx_empty) begin
                        state <= IDLE;
                    end
                end

            endcase

        end
    end
endmodule


// module tb;

//     reg         clk, rst;
//     wire [3:0]  tx;
//     reg [7:0]   tx_d;
//     wire        tx_ack;

//     TX_INTF #(
//         .NOC_WID(4)
//     ) i_TX_INTF (
//         .clk(clk),
//         .rst(rst),
//         .tx(tx),
//         .tx_d(tx_d),
//         .tx_ack(tx_ack)
//     );

//     always #5 clk = ~clk;

//     initial begin
//         $dumpfile("dump.vcd");
//         $dumpvars(0, tb);

//         clk = 0;
//         rst = 1;

//         tx_d = 0;
//         #21;

//         rst = 0;
//         tx_d = 8'b10100101;
//         #400;
//         tx_d = 0;

//         #200;
//         $finish;

//     end


// endmodule
module WB_INTF #(
    parameter WB_WID = 32,
    parameter NOC_WID = 16
) (

    input                       en,             // mux enabled

    input                       wb_clk_i,
    input                       wb_rst_i,
    input                       wbs_stb_i,
    input                       wbs_cyc_i,
    input                       wbs_we_i,
    input [(WB_WID/8)-1:0]      wbs_sel_i,
    input [WB_WID-1:0]          wbs_dat_i,
    input [WB_WID-1:0]          wbs_adr_i,
    output reg                  wbs_ack_o,
    output reg [WB_WID-1:0]     wbs_dat_o,

    // To repeaters
    output reg [WB_WID-1:0]     repeat_dat,
    output reg [WB_WID-1:0]     repeat_adr,

    // To NoC wrappers
    output reg [NOC_WID-1:0]    noc_rx,
    output reg [7:0]            noc_rx_bits,
    output reg                  noc_rx_toggle,
    input [NOC_WID-1:0]         noc_tx,
    output reg                  noc_tx_toggle
);

    localparam DEFAULT_ADDR = 32'hFFFFFFFF;     // Out of bounds for all PEs

    localparam NOC_RX_ADDR = 32'hFFFF0000;
    localparam NOC_TX_ADDR = 32'hFFFF0004;

    always @(posedge wb_clk_i) begin
        if (wb_rst_i | ~en) begin
            wbs_dat_o <= 0;
            repeat_dat <= 0;
            repeat_adr <= DEFAULT_ADDR;
            noc_rx <= 0;
            noc_rx_bits <= 0;
            noc_rx_toggle <= 0;
            noc_tx_toggle <= 0;
        end else if (wbs_stb_i && wbs_cyc_i) begin
            repeat_dat <= 0;
            repeat_adr <= DEFAULT_ADDR;
            noc_rx <= 0;
            wbs_dat_o <= 0;

            if (wbs_we_i && ~wbs_ack_o) begin                   // Write: stolen from Charles
                repeat_dat <= wbs_dat_i;
                repeat_adr <= wbs_adr_i;

                if (wbs_adr_i == NOC_RX_ADDR) begin
                    noc_rx <= wbs_dat_i[NOC_WID-1:0];
                    noc_rx_bits <= wbs_dat_i[NOC_WID+7:NOC_WID];
                    noc_rx_toggle <= ~noc_rx_toggle;
                end

            end else if (~wbs_we_i && ~wbs_ack_o) begin         // Read: stolen from Charles
                if (wbs_adr_i == NOC_TX_ADDR) begin
                    wbs_dat_o <= noc_tx;
                    noc_tx_toggle <= ~noc_tx_toggle;
                end
            end
        end
    end

    always @(posedge wb_clk_i) begin
        if (wb_rst_i | ~en) begin
            wbs_ack_o <= 0;
        end else begin
            wbs_ack_o <= (wbs_stb_i && wbs_cyc_i);              // We can process immediately: stolen from Charles
        end
    end
endmodule
module bus_repeater #(
    parameter WB_WID = 32,
    parameter NOC_WID = 16,
    parameter REGIONAL_ADDR_WID = 9,
    parameter FANOUT = 32,
    parameter ADDR_LO = 32'h0,
    parameter ADDR_HI = 32'b001000000000        // non-inclusive
) (

    input                               clk,
    input                               rst,
    input [WB_WID-1:0]                  in_dat,
    input [WB_WID-1:0]                  in_adr,

    output reg [NOC_WID-1:0]            out_dat,
    output reg [REGIONAL_ADDR_WID-1:0]  out_adr
);

    always @(posedge clk) begin
        if (rst) begin
            out_dat <= 0;
            out_adr <= 1;       // Note: address 0 actually maps to a PE
        end else begin
            if (in_adr >= ADDR_LO && in_adr < ADDR_HI) begin
                out_dat <= in_dat[NOC_WID-1:0];
                out_adr <= in_adr-ADDR_LO;
            end else begin
                out_dat <= 0;
                out_adr <= 1;
            end
        end
    end

endmodule


// module tb;

//     reg clk, rst;
//     reg [31:0] in_dat, in_adr;
//     wire [15:0] out_dat;
//     wire [8:0] out_adr;

//     bus_repeater i_bus_repeater (
//         .clk    (clk    ),
//         .rst    (rst    ),
//         .in_dat (in_dat ),
//         .in_adr (in_adr ),
//         .out_dat(out_dat),
//         .out_adr(out_adr)
//     );

//     always #5 clk = ~clk;

//     initial begin
//         $dumpfile("dump.vcd");
//         $dumpvars(0, tb);

//         clk = 0;
//         rst = 1;

//         in_dat = 0;
//         in_adr = 0;

//         #21;
//         rst = 0;
//         in_dat = 1;
//         in_adr = 0;

//         #10;
//         in_dat = 2;
//         in_adr = 32'b001000000000;

//         #200;
//         $finish;
//     end

// endmodule
module leaf #(
    parameter NOC_WID = 16,
    parameter INIT_MSG = 32'b01010101010101010101010101010101
)(
    input       clk,
    input       rst,

    input                       rx_pr,
    input [1:0]                 rx_pd,
    output wire                 rx_pa,

    output wire [2*NOC_WID-1:0] tx_pd,
    input                       tx_pa
);


    localparam [1:0]
        RX_IDLE = 2'b00,
        RX_ACK_HI = 2'b01,
        RX_ACK_LO = 2'b10;

    reg [1:0] rx_state;

    wire rx_valid;

    assign rx_pa = (rx_state == RX_ACK_HI) ? 1 : 0;
    assign rx_valid = rx_pd[0] || rx_pd[1];

    always @(posedge clk) begin
        if (rst) begin
            rx_state <= RX_IDLE;
        end else begin
            case (rx_state)
                RX_IDLE: begin
                    if (rx_pr) begin
                        rx_state <= RX_ACK_HI;
                    end
                end
                RX_ACK_HI: begin
                    if (~rx_pr) begin
                        rx_state <= RX_IDLE;
                    end else if (rx_valid) begin
                        rx_state <= RX_ACK_LO;
                    end
                end
                RX_ACK_LO: begin
                    if (~rx_valid) begin
                        rx_state <= RX_ACK_HI;
                    end
                end
            endcase
        end
    end

    localparam [1:0]
        TX_IDLE = 2'b00,
        TX_SEND = 2'b01,
        TX_NOSEND = 2'b10;

    reg [1:0] tx_state;

    assign tx_pd = (tx_state == TX_SEND) ? INIT_MSG : 0;

    always @(posedge clk) begin
        if (rst) begin
            tx_state <= TX_IDLE;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    if (rx_state == RX_ACK_HI) begin
                        tx_state <= TX_SEND;
                    end
                end
                TX_SEND: begin
                    if (tx_pa) begin
                        tx_state <= TX_NOSEND;
                    end
                end
                TX_NOSEND: begin
                    if (rx_state == RX_IDLE) begin
                        tx_state <= TX_IDLE;
                    end
                end
            endcase
        end
    end


endmodule
module Top (
    input               wb_clk_i,
    input               wb_rst_i,
    input               wbs_stb_i,
    input               wbs_cyc_i,
    input               wbs_we_i,
    input [3:0]         wbs_sel_i,
    input [31:0]        wbs_dat_i,
    input [31:0]        wbs_adr_i,
    output wire         wbs_ack_o,
    output wire [31:0]  wbs_dat_o
);

    wire [31:0]     br_dat, br_adr;
    wire [15:0]     rx, tx;
    wire [7:0]      rx_bits;
    wire            rx_toggle, tx_toggle;
    WB_INTF #(
        .WB_WID(32),
        .NOC_WID(16)
    ) i_WB_INTF (
        .en           (1'b1),
        .wb_clk_i     (wb_clk_i),
        .wb_rst_i     (wb_rst_i),
        .wbs_stb_i    (wbs_stb_i),
        .wbs_cyc_i    (wbs_cyc_i),
        .wbs_we_i     (wbs_we_i),
        .wbs_sel_i    (wbs_sel_i),
        .wbs_dat_i    (wbs_dat_i),
        .wbs_adr_i    (wbs_adr_i),
        .wbs_ack_o    (wbs_ack_o),
        .wbs_dat_o    (wbs_dat_o),
        .repeat_dat   (br_dat),
        .repeat_adr   (br_adr),
        .noc_rx       (rx),
        .noc_rx_bits  (rx_bits),
        .noc_rx_toggle(rx_toggle),
        .noc_tx       (tx),
        .noc_tx_toggle(tx_toggle)
    );


    wire            top_rx_req, top_rx_ack;
    wire [1:0]      top_rx_d;
    RX_INTF #(
        .NOC_WID(16)
    ) i_RX_INTF (
        .clk      (wb_clk_i),
        .rst      (wb_rst_i),
        .rx       (rx),
        .rx_bits  (rx_bits),
        .rx_toggle(rx_toggle),
        .rx_req   (top_rx_req),
        .rx_d     (top_rx_d),
        .rx_ack   (top_rx_ack)
    );

    wire [31:0]     top_tx_d;
    wire            top_tx_ack;
    TX_INTF #(
        .NOC_WID(16)
    ) i_TX_INTF (
        .clk   (wb_clk_i),
        .rst   (wb_rst_i),
        .tx    (tx),
        .tx_d  (top_tx_d),
        .tx_ack(top_tx_ack)
    );

    wire [15:0]  br0_dat;
    wire [10:0]  br0_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h30000000),
        .ADDR_HI          (32'h30000078)
    ) br0 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br0_dat),  .out_adr(br0_adr)
    );

    wire [15:0]  br1_dat;
    wire [10:0]  br1_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h30000078),
        .ADDR_HI          (32'h300000cc)
    ) br1 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br1_dat),  .out_adr(br1_adr)
    );

    wire [15:0]  br2_dat;
    wire [10:0]  br2_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h300000cc),
        .ADDR_HI          (32'h3000012c)
    ) br2 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br2_dat),  .out_adr(br2_adr)
    );

    wire [15:0]  br3_dat;
    wire [10:0]  br3_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h3000012c),
        .ADDR_HI          (32'h30000180)
    ) br3 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br3_dat),  .out_adr(br3_adr)
    );

    wire [15:0]  br4_dat;
    wire [10:0]  br4_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h30000180),
        .ADDR_HI          (32'h300001ec)
    ) br4 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br4_dat),  .out_adr(br4_adr)
    );

    wire [15:0]  br5_dat;
    wire [10:0]  br5_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h300001ec),
        .ADDR_HI          (32'h30000240)
    ) br5 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br5_dat),  .out_adr(br5_adr)
    );

    wire [15:0]  br6_dat;
    wire [10:0]  br6_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h30000240),
        .ADDR_HI          (32'h300002a0)
    ) br6 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br6_dat),  .out_adr(br6_adr)
    );

    wire [15:0]  br7_dat;
    wire [10:0]  br7_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h300002a0),
        .ADDR_HI          (32'h300002f4)
    ) br7 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br7_dat),  .out_adr(br7_adr)
    );


    wire            r_rxr, r_rxa, r_txa;
    wire [1:0]      r_rxd;
    wire [31:0]     r_txd;
    wire            rl_rxr, rl_rxa, rl_txa;
    wire [1:0]      rl_rxd;
    wire [31:0]     rl_txd;
    wire            rll_rxr, rll_rxa, rll_txa;
    wire [1:0]      rll_rxd;
    wire [31:0]     rll_txd;
    wire            rlll_rxr, rlll_rxa, rlll_txa;
    wire [1:0]      rlll_rxd;
    wire [31:0]     rlll_txd;
    wire            rllll_rxr, rllll_rxa, rllll_txa;
    wire [1:0]      rllll_rxd;
    wire [31:0]     rllll_txd;
    wire            rlllll_rxr, rlllll_rxa, rlllll_txa;
    wire [1:0]      rlllll_rxd;
    wire [31:0]     rlllll_txd;
    wire            rllllll_rxr, rllllll_rxa, rllllll_txa;
    wire [1:0]      rllllll_rxd;
    wire [31:0]     rllllll_txd;
    wire            rlllllr_rxr, rlllllr_rxa, rlllllr_txa;
    wire [1:0]      rlllllr_rxd;
    wire [31:0]     rlllllr_txd;
    wire            rllllr_rxr, rllllr_rxa, rllllr_txa;
    wire [1:0]      rllllr_rxd;
    wire [31:0]     rllllr_txd;
    wire            rllllrl_rxr, rllllrl_rxa, rllllrl_txa;
    wire [1:0]      rllllrl_rxd;
    wire [31:0]     rllllrl_txd;
    wire            rllllrr_rxr, rllllrr_rxa, rllllrr_txa;
    wire [1:0]      rllllrr_rxd;
    wire [31:0]     rllllrr_txd;
    wire            rlllr_rxr, rlllr_rxa, rlllr_txa;
    wire [1:0]      rlllr_rxd;
    wire [31:0]     rlllr_txd;
    wire            rlllrl_rxr, rlllrl_rxa, rlllrl_txa;
    wire [1:0]      rlllrl_rxd;
    wire [31:0]     rlllrl_txd;
    wire            rlllrll_rxr, rlllrll_rxa, rlllrll_txa;
    wire [1:0]      rlllrll_rxd;
    wire [31:0]     rlllrll_txd;
    wire            rlllrlr_rxr, rlllrlr_rxa, rlllrlr_txa;
    wire [1:0]      rlllrlr_rxd;
    wire [31:0]     rlllrlr_txd;
    wire            rlllrr_rxr, rlllrr_rxa, rlllrr_txa;
    wire [1:0]      rlllrr_rxd;
    wire [31:0]     rlllrr_txd;
    wire            rlllrrl_rxr, rlllrrl_rxa, rlllrrl_txa;
    wire [1:0]      rlllrrl_rxd;
    wire [31:0]     rlllrrl_txd;
    wire            rlllrrr_rxr, rlllrrr_rxa, rlllrrr_txa;
    wire [1:0]      rlllrrr_rxd;
    wire [31:0]     rlllrrr_txd;
    wire            rllr_rxr, rllr_rxa, rllr_txa;
    wire [1:0]      rllr_rxd;
    wire [31:0]     rllr_txd;
    wire            rllrl_rxr, rllrl_rxa, rllrl_txa;
    wire [1:0]      rllrl_rxd;
    wire [31:0]     rllrl_txd;
    wire            rllrll_rxr, rllrll_rxa, rllrll_txa;
    wire [1:0]      rllrll_rxd;
    wire [31:0]     rllrll_txd;
    wire            rllrlll_rxr, rllrlll_rxa, rllrlll_txa;
    wire [1:0]      rllrlll_rxd;
    wire [31:0]     rllrlll_txd;
    wire            rllrllr_rxr, rllrllr_rxa, rllrllr_txa;
    wire [1:0]      rllrllr_rxd;
    wire [31:0]     rllrllr_txd;
    wire            rllrlr_rxr, rllrlr_rxa, rllrlr_txa;
    wire [1:0]      rllrlr_rxd;
    wire [31:0]     rllrlr_txd;
    wire            rllrlrl_rxr, rllrlrl_rxa, rllrlrl_txa;
    wire [1:0]      rllrlrl_rxd;
    wire [31:0]     rllrlrl_txd;
    wire            rllrlrr_rxr, rllrlrr_rxa, rllrlrr_txa;
    wire [1:0]      rllrlrr_rxd;
    wire [31:0]     rllrlrr_txd;
    wire            rllrr_rxr, rllrr_rxa, rllrr_txa;
    wire [1:0]      rllrr_rxd;
    wire [31:0]     rllrr_txd;
    wire            rllrrl_rxr, rllrrl_rxa, rllrrl_txa;
    wire [1:0]      rllrrl_rxd;
    wire [31:0]     rllrrl_txd;
    wire            rllrrll_rxr, rllrrll_rxa, rllrrll_txa;
    wire [1:0]      rllrrll_rxd;
    wire [31:0]     rllrrll_txd;
    wire            rllrrlr_rxr, rllrrlr_rxa, rllrrlr_txa;
    wire [1:0]      rllrrlr_rxd;
    wire [31:0]     rllrrlr_txd;
    wire            rllrrr_rxr, rllrrr_rxa, rllrrr_txa;
    wire [1:0]      rllrrr_rxd;
    wire [31:0]     rllrrr_txd;
    wire            rllrrrl_rxr, rllrrrl_rxa, rllrrrl_txa;
    wire [1:0]      rllrrrl_rxd;
    wire [31:0]     rllrrrl_txd;
    wire            rllrrrr_rxr, rllrrrr_rxa, rllrrrr_txa;
    wire [1:0]      rllrrrr_rxd;
    wire [31:0]     rllrrrr_txd;
    wire            rlr_rxr, rlr_rxa, rlr_txa;
    wire [1:0]      rlr_rxd;
    wire [31:0]     rlr_txd;
    wire            rlrl_rxr, rlrl_rxa, rlrl_txa;
    wire [1:0]      rlrl_rxd;
    wire [31:0]     rlrl_txd;
    wire            rlrll_rxr, rlrll_rxa, rlrll_txa;
    wire [1:0]      rlrll_rxd;
    wire [31:0]     rlrll_txd;
    wire            rlrlll_rxr, rlrlll_rxa, rlrlll_txa;
    wire [1:0]      rlrlll_rxd;
    wire [31:0]     rlrlll_txd;
    wire            rlrllll_rxr, rlrllll_rxa, rlrllll_txa;
    wire [1:0]      rlrllll_rxd;
    wire [31:0]     rlrllll_txd;
    wire            rlrlllr_rxr, rlrlllr_rxa, rlrlllr_txa;
    wire [1:0]      rlrlllr_rxd;
    wire [31:0]     rlrlllr_txd;
    wire            rlrllr_rxr, rlrllr_rxa, rlrllr_txa;
    wire [1:0]      rlrllr_rxd;
    wire [31:0]     rlrllr_txd;
    wire            rlrllrl_rxr, rlrllrl_rxa, rlrllrl_txa;
    wire [1:0]      rlrllrl_rxd;
    wire [31:0]     rlrllrl_txd;
    wire            rlrllrr_rxr, rlrllrr_rxa, rlrllrr_txa;
    wire [1:0]      rlrllrr_rxd;
    wire [31:0]     rlrllrr_txd;
    wire            rlrlr_rxr, rlrlr_rxa, rlrlr_txa;
    wire [1:0]      rlrlr_rxd;
    wire [31:0]     rlrlr_txd;
    wire            rlrlrl_rxr, rlrlrl_rxa, rlrlrl_txa;
    wire [1:0]      rlrlrl_rxd;
    wire [31:0]     rlrlrl_txd;
    wire            rlrlrll_rxr, rlrlrll_rxa, rlrlrll_txa;
    wire [1:0]      rlrlrll_rxd;
    wire [31:0]     rlrlrll_txd;
    wire            rlrlrlr_rxr, rlrlrlr_rxa, rlrlrlr_txa;
    wire [1:0]      rlrlrlr_rxd;
    wire [31:0]     rlrlrlr_txd;
    wire            rlrlrr_rxr, rlrlrr_rxa, rlrlrr_txa;
    wire [1:0]      rlrlrr_rxd;
    wire [31:0]     rlrlrr_txd;
    wire            rlrlrrl_rxr, rlrlrrl_rxa, rlrlrrl_txa;
    wire [1:0]      rlrlrrl_rxd;
    wire [31:0]     rlrlrrl_txd;
    wire            rlrlrrr_rxr, rlrlrrr_rxa, rlrlrrr_txa;
    wire [1:0]      rlrlrrr_rxd;
    wire [31:0]     rlrlrrr_txd;
    wire            rlrr_rxr, rlrr_rxa, rlrr_txa;
    wire [1:0]      rlrr_rxd;
    wire [31:0]     rlrr_txd;
    wire            rlrrl_rxr, rlrrl_rxa, rlrrl_txa;
    wire [1:0]      rlrrl_rxd;
    wire [31:0]     rlrrl_txd;
    wire            rlrrll_rxr, rlrrll_rxa, rlrrll_txa;
    wire [1:0]      rlrrll_rxd;
    wire [31:0]     rlrrll_txd;
    wire            rlrrlll_rxr, rlrrlll_rxa, rlrrlll_txa;
    wire [1:0]      rlrrlll_rxd;
    wire [31:0]     rlrrlll_txd;
    wire            rlrrllr_rxr, rlrrllr_rxa, rlrrllr_txa;
    wire [1:0]      rlrrllr_rxd;
    wire [31:0]     rlrrllr_txd;
    wire            rlrrlr_rxr, rlrrlr_rxa, rlrrlr_txa;
    wire [1:0]      rlrrlr_rxd;
    wire [31:0]     rlrrlr_txd;
    wire            rlrrlrl_rxr, rlrrlrl_rxa, rlrrlrl_txa;
    wire [1:0]      rlrrlrl_rxd;
    wire [31:0]     rlrrlrl_txd;
    wire            rlrrlrr_rxr, rlrrlrr_rxa, rlrrlrr_txa;
    wire [1:0]      rlrrlrr_rxd;
    wire [31:0]     rlrrlrr_txd;
    wire            rlrrr_rxr, rlrrr_rxa, rlrrr_txa;
    wire [1:0]      rlrrr_rxd;
    wire [31:0]     rlrrr_txd;
    wire            rlrrrl_rxr, rlrrrl_rxa, rlrrrl_txa;
    wire [1:0]      rlrrrl_rxd;
    wire [31:0]     rlrrrl_txd;
    wire            rlrrrll_rxr, rlrrrll_rxa, rlrrrll_txa;
    wire [1:0]      rlrrrll_rxd;
    wire [31:0]     rlrrrll_txd;
    wire            rlrrrlr_rxr, rlrrrlr_rxa, rlrrrlr_txa;
    wire [1:0]      rlrrrlr_rxd;
    wire [31:0]     rlrrrlr_txd;
    wire            rlrrrr_rxr, rlrrrr_rxa, rlrrrr_txa;
    wire [1:0]      rlrrrr_rxd;
    wire [31:0]     rlrrrr_txd;
    wire            rlrrrrl_rxr, rlrrrrl_rxa, rlrrrrl_txa;
    wire [1:0]      rlrrrrl_rxd;
    wire [31:0]     rlrrrrl_txd;
    wire            rlrrrrr_rxr, rlrrrrr_rxa, rlrrrrr_txa;
    wire [1:0]      rlrrrrr_rxd;
    wire [31:0]     rlrrrrr_txd;
    wire            rr_rxr, rr_rxa, rr_txa;
    wire [1:0]      rr_rxd;
    wire [31:0]     rr_txd;
    wire            rrl_rxr, rrl_rxa, rrl_txa;
    wire [1:0]      rrl_rxd;
    wire [31:0]     rrl_txd;
    wire            rrll_rxr, rrll_rxa, rrll_txa;
    wire [1:0]      rrll_rxd;
    wire [31:0]     rrll_txd;
    wire            rrlll_rxr, rrlll_rxa, rrlll_txa;
    wire [1:0]      rrlll_rxd;
    wire [31:0]     rrlll_txd;
    wire            rrllll_rxr, rrllll_rxa, rrllll_txa;
    wire [1:0]      rrllll_rxd;
    wire [31:0]     rrllll_txd;
    wire            rrlllll_rxr, rrlllll_rxa, rrlllll_txa;
    wire [1:0]      rrlllll_rxd;
    wire [31:0]     rrlllll_txd;
    wire            rrllllr_rxr, rrllllr_rxa, rrllllr_txa;
    wire [1:0]      rrllllr_rxd;
    wire [31:0]     rrllllr_txd;
    wire            rrlllr_rxr, rrlllr_rxa, rrlllr_txa;
    wire [1:0]      rrlllr_rxd;
    wire [31:0]     rrlllr_txd;
    wire            rrlllrl_rxr, rrlllrl_rxa, rrlllrl_txa;
    wire [1:0]      rrlllrl_rxd;
    wire [31:0]     rrlllrl_txd;
    wire            rrlllrr_rxr, rrlllrr_rxa, rrlllrr_txa;
    wire [1:0]      rrlllrr_rxd;
    wire [31:0]     rrlllrr_txd;
    wire            rrllr_rxr, rrllr_rxa, rrllr_txa;
    wire [1:0]      rrllr_rxd;
    wire [31:0]     rrllr_txd;
    wire            rrllrl_rxr, rrllrl_rxa, rrllrl_txa;
    wire [1:0]      rrllrl_rxd;
    wire [31:0]     rrllrl_txd;
    wire            rrllrll_rxr, rrllrll_rxa, rrllrll_txa;
    wire [1:0]      rrllrll_rxd;
    wire [31:0]     rrllrll_txd;
    wire            rrllrlr_rxr, rrllrlr_rxa, rrllrlr_txa;
    wire [1:0]      rrllrlr_rxd;
    wire [31:0]     rrllrlr_txd;
    wire            rrllrr_rxr, rrllrr_rxa, rrllrr_txa;
    wire [1:0]      rrllrr_rxd;
    wire [31:0]     rrllrr_txd;
    wire            rrllrrl_rxr, rrllrrl_rxa, rrllrrl_txa;
    wire [1:0]      rrllrrl_rxd;
    wire [31:0]     rrllrrl_txd;
    wire            rrllrrr_rxr, rrllrrr_rxa, rrllrrr_txa;
    wire [1:0]      rrllrrr_rxd;
    wire [31:0]     rrllrrr_txd;
    wire            rrlr_rxr, rrlr_rxa, rrlr_txa;
    wire [1:0]      rrlr_rxd;
    wire [31:0]     rrlr_txd;
    wire            rrlrl_rxr, rrlrl_rxa, rrlrl_txa;
    wire [1:0]      rrlrl_rxd;
    wire [31:0]     rrlrl_txd;
    wire            rrlrll_rxr, rrlrll_rxa, rrlrll_txa;
    wire [1:0]      rrlrll_rxd;
    wire [31:0]     rrlrll_txd;
    wire            rrlrlll_rxr, rrlrlll_rxa, rrlrlll_txa;
    wire [1:0]      rrlrlll_rxd;
    wire [31:0]     rrlrlll_txd;
    wire            rrlrllr_rxr, rrlrllr_rxa, rrlrllr_txa;
    wire [1:0]      rrlrllr_rxd;
    wire [31:0]     rrlrllr_txd;
    wire            rrlrlr_rxr, rrlrlr_rxa, rrlrlr_txa;
    wire [1:0]      rrlrlr_rxd;
    wire [31:0]     rrlrlr_txd;
    wire            rrlrlrl_rxr, rrlrlrl_rxa, rrlrlrl_txa;
    wire [1:0]      rrlrlrl_rxd;
    wire [31:0]     rrlrlrl_txd;
    wire            rrlrlrr_rxr, rrlrlrr_rxa, rrlrlrr_txa;
    wire [1:0]      rrlrlrr_rxd;
    wire [31:0]     rrlrlrr_txd;
    wire            rrlrr_rxr, rrlrr_rxa, rrlrr_txa;
    wire [1:0]      rrlrr_rxd;
    wire [31:0]     rrlrr_txd;
    wire            rrlrrl_rxr, rrlrrl_rxa, rrlrrl_txa;
    wire [1:0]      rrlrrl_rxd;
    wire [31:0]     rrlrrl_txd;
    wire            rrlrrll_rxr, rrlrrll_rxa, rrlrrll_txa;
    wire [1:0]      rrlrrll_rxd;
    wire [31:0]     rrlrrll_txd;
    wire            rrlrrlr_rxr, rrlrrlr_rxa, rrlrrlr_txa;
    wire [1:0]      rrlrrlr_rxd;
    wire [31:0]     rrlrrlr_txd;
    wire            rrlrrr_rxr, rrlrrr_rxa, rrlrrr_txa;
    wire [1:0]      rrlrrr_rxd;
    wire [31:0]     rrlrrr_txd;
    wire            rrlrrrl_rxr, rrlrrrl_rxa, rrlrrrl_txa;
    wire [1:0]      rrlrrrl_rxd;
    wire [31:0]     rrlrrrl_txd;
    wire            rrlrrrr_rxr, rrlrrrr_rxa, rrlrrrr_txa;
    wire [1:0]      rrlrrrr_rxd;
    wire [31:0]     rrlrrrr_txd;
    wire            rrr_rxr, rrr_rxa, rrr_txa;
    wire [1:0]      rrr_rxd;
    wire [31:0]     rrr_txd;
    wire            rrrl_rxr, rrrl_rxa, rrrl_txa;
    wire [1:0]      rrrl_rxd;
    wire [31:0]     rrrl_txd;
    wire            rrrll_rxr, rrrll_rxa, rrrll_txa;
    wire [1:0]      rrrll_rxd;
    wire [31:0]     rrrll_txd;
    wire            rrrlll_rxr, rrrlll_rxa, rrrlll_txa;
    wire [1:0]      rrrlll_rxd;
    wire [31:0]     rrrlll_txd;
    wire            rrrllll_rxr, rrrllll_rxa, rrrllll_txa;
    wire [1:0]      rrrllll_rxd;
    wire [31:0]     rrrllll_txd;
    wire            rrrlllr_rxr, rrrlllr_rxa, rrrlllr_txa;
    wire [1:0]      rrrlllr_rxd;
    wire [31:0]     rrrlllr_txd;
    wire            rrrllr_rxr, rrrllr_rxa, rrrllr_txa;
    wire [1:0]      rrrllr_rxd;
    wire [31:0]     rrrllr_txd;
    wire            rrrllrl_rxr, rrrllrl_rxa, rrrllrl_txa;
    wire [1:0]      rrrllrl_rxd;
    wire [31:0]     rrrllrl_txd;
    wire            rrrllrr_rxr, rrrllrr_rxa, rrrllrr_txa;
    wire [1:0]      rrrllrr_rxd;
    wire [31:0]     rrrllrr_txd;
    wire            rrrlr_rxr, rrrlr_rxa, rrrlr_txa;
    wire [1:0]      rrrlr_rxd;
    wire [31:0]     rrrlr_txd;
    wire            rrrlrl_rxr, rrrlrl_rxa, rrrlrl_txa;
    wire [1:0]      rrrlrl_rxd;
    wire [31:0]     rrrlrl_txd;
    wire            rrrlrll_rxr, rrrlrll_rxa, rrrlrll_txa;
    wire [1:0]      rrrlrll_rxd;
    wire [31:0]     rrrlrll_txd;
    wire            rrrlrlr_rxr, rrrlrlr_rxa, rrrlrlr_txa;
    wire [1:0]      rrrlrlr_rxd;
    wire [31:0]     rrrlrlr_txd;
    wire            rrrlrr_rxr, rrrlrr_rxa, rrrlrr_txa;
    wire [1:0]      rrrlrr_rxd;
    wire [31:0]     rrrlrr_txd;
    wire            rrrlrrl_rxr, rrrlrrl_rxa, rrrlrrl_txa;
    wire [1:0]      rrrlrrl_rxd;
    wire [31:0]     rrrlrrl_txd;
    wire            rrrlrrr_rxr, rrrlrrr_rxa, rrrlrrr_txa;
    wire [1:0]      rrrlrrr_rxd;
    wire [31:0]     rrrlrrr_txd;
    wire            rrrr_rxr, rrrr_rxa, rrrr_txa;
    wire [1:0]      rrrr_rxd;
    wire [31:0]     rrrr_txd;
    wire            rrrrl_rxr, rrrrl_rxa, rrrrl_txa;
    wire [1:0]      rrrrl_rxd;
    wire [31:0]     rrrrl_txd;
    wire            rrrrll_rxr, rrrrll_rxa, rrrrll_txa;
    wire [1:0]      rrrrll_rxd;
    wire [31:0]     rrrrll_txd;
    wire            rrrrlll_rxr, rrrrlll_rxa, rrrrlll_txa;
    wire [1:0]      rrrrlll_rxd;
    wire [31:0]     rrrrlll_txd;
    wire            rrrrllr_rxr, rrrrllr_rxa, rrrrllr_txa;
    wire [1:0]      rrrrllr_rxd;
    wire [31:0]     rrrrllr_txd;
    wire            rrrrlr_rxr, rrrrlr_rxa, rrrrlr_txa;
    wire [1:0]      rrrrlr_rxd;
    wire [31:0]     rrrrlr_txd;
    wire            rrrrlrl_rxr, rrrrlrl_rxa, rrrrlrl_txa;
    wire [1:0]      rrrrlrl_rxd;
    wire [31:0]     rrrrlrl_txd;
    wire            rrrrlrr_rxr, rrrrlrr_rxa, rrrrlrr_txa;
    wire [1:0]      rrrrlrr_rxd;
    wire [31:0]     rrrrlrr_txd;
    wire            rrrrr_rxr, rrrrr_rxa, rrrrr_txa;
    wire [1:0]      rrrrr_rxd;
    wire [31:0]     rrrrr_txd;
    wire            rrrrrl_rxr, rrrrrl_rxa, rrrrrl_txa;
    wire [1:0]      rrrrrl_rxd;
    wire [31:0]     rrrrrl_txd;
    wire            rrrrrll_rxr, rrrrrll_rxa, rrrrrll_txa;
    wire [1:0]      rrrrrll_rxd;
    wire [31:0]     rrrrrll_txd;
    wire            rrrrrlr_rxr, rrrrrlr_rxa, rrrrrlr_txa;
    wire [1:0]      rrrrrlr_rxd;
    wire [31:0]     rrrrrlr_txd;
    wire            rrrrrr_rxr, rrrrrr_rxa, rrrrrr_txa;
    wire [1:0]      rrrrrr_rxd;
    wire [31:0]     rrrrrr_txd;
    wire            rrrrrrl_rxr, rrrrrrl_rxa, rrrrrrl_txa;
    wire [1:0]      rrrrrrl_rxd;
    wire [31:0]     rrrrrrl_txd;
    wire            rrrrrrr_rxr, rrrrrrr_rxa, rrrrrrr_txa;
    wire [1:0]      rrrrrrr_rxd;
    wire [31:0]     rrrrrrr_txd;


    assign r_rxr = top_rx_req;
    assign r_rxd = top_rx_d;
    assign top_rx_ack = r_rxa;
    assign top_tx_d = r_txd;
    assign r_txa = top_tx_ack;
    PE_right r (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h0),
        .rx_pr(r_rxr), .rx_pd(r_rxd), .rx_pa(r_rxa),
        .rx_c0r(rl_rxr), .rx_c0d(rl_rxd), .rx_c0a(rl_rxa),
        .rx_c1r(rr_rxr), .rx_c1d(rr_rxd), .rx_c1a(rr_rxa),
        .tx_c0d(rl_txd), .tx_c0a(rl_txa),
        .tx_c1d(rr_txd), .tx_c1a(rr_txa),
        .tx_pd(r_txd), .tx_pa(r_txa)
    );
    PE_down rl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'hc),
        .rx_pr(rl_rxr), .rx_pd(rl_rxd), .rx_pa(rl_rxa),
        .rx_c0r(rll_rxr), .rx_c0d(rll_rxd), .rx_c0a(rll_rxa),
        .rx_c1r(rlr_rxr), .rx_c1d(rlr_rxd), .rx_c1a(rlr_rxa),
        .tx_c0d(rll_txd), .tx_c0a(rll_txa),
        .tx_c1d(rlr_txd), .tx_c1a(rlr_txa),
        .tx_pd(rl_txd), .tx_pa(rl_txa)
    );
    PE_right rll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h18),
        .rx_pr(rll_rxr), .rx_pd(rll_rxd), .rx_pa(rll_rxa),
        .rx_c0r(rlll_rxr), .rx_c0d(rlll_rxd), .rx_c0a(rlll_rxa),
        .rx_c1r(rllr_rxr), .rx_c1d(rllr_rxd), .rx_c1a(rllr_rxa),
        .tx_c0d(rlll_txd), .tx_c0a(rlll_txa),
        .tx_c1d(rllr_txd), .tx_c1a(rllr_txa),
        .tx_pd(rll_txd), .tx_pa(rll_txa)
    );
    PE_down rlll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h24),
        .rx_pr(rlll_rxr), .rx_pd(rlll_rxd), .rx_pa(rlll_rxa),
        .rx_c0r(rllll_rxr), .rx_c0d(rllll_rxd), .rx_c0a(rllll_rxa),
        .rx_c1r(rlllr_rxr), .rx_c1d(rlllr_rxd), .rx_c1a(rlllr_rxa),
        .tx_c0d(rllll_txd), .tx_c0a(rllll_txa),
        .tx_c1d(rlllr_txd), .tx_c1a(rlllr_txa),
        .tx_pd(rlll_txd), .tx_pa(rlll_txa)
    );
    PE_right rllll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h30),
        .rx_pr(rllll_rxr), .rx_pd(rllll_rxd), .rx_pa(rllll_rxa),
        .rx_c0r(rlllll_rxr), .rx_c0d(rlllll_rxd), .rx_c0a(rlllll_rxa),
        .rx_c1r(rllllr_rxr), .rx_c1d(rllllr_rxd), .rx_c1a(rllllr_rxa),
        .tx_c0d(rlllll_txd), .tx_c0a(rlllll_txa),
        .tx_c1d(rllllr_txd), .tx_c1a(rllllr_txa),
        .tx_pd(rllll_txd), .tx_pa(rllll_txa)
    );
    PE_down rlllll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h3c),
        .rx_pr(rlllll_rxr), .rx_pd(rlllll_rxd), .rx_pa(rlllll_rxa),
        .rx_c0r(rllllll_rxr), .rx_c0d(rllllll_rxd), .rx_c0a(rllllll_rxa),
        .rx_c1r(rlllllr_rxr), .rx_c1d(rlllllr_rxd), .rx_c1a(rlllllr_rxa),
        .tx_c0d(rllllll_txd), .tx_c0a(rllllll_txa),
        .tx_c1d(rlllllr_txd), .tx_c1a(rlllllr_txa),
        .tx_pd(rlllll_txd), .tx_pa(rlllll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllllll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllllll_rxr),
        .rx_pd(rllllll_rxd),
        .rx_pa(rllllll_rxa),
        .tx_pd(rllllll_txd),
        .tx_pa(rllllll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlllllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlllllr_rxr),
        .rx_pd(rlllllr_rxd),
        .rx_pa(rlllllr_rxa),
        .tx_pd(rlllllr_txd),
        .tx_pa(rlllllr_txa)
    );
    PE_up rllllr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h48),
        .rx_pr(rllllr_rxr), .rx_pd(rllllr_rxd), .rx_pa(rllllr_rxa),
        .rx_c0r(rllllrl_rxr), .rx_c0d(rllllrl_rxd), .rx_c0a(rllllrl_rxa),
        .rx_c1r(rllllrr_rxr), .rx_c1d(rllllrr_rxd), .rx_c1a(rllllrr_rxa),
        .tx_c0d(rllllrl_txd), .tx_c0a(rllllrl_txa),
        .tx_c1d(rllllrr_txd), .tx_c1a(rllllrr_txa),
        .tx_pd(rllllr_txd), .tx_pa(rllllr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllllrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllllrl_rxr),
        .rx_pd(rllllrl_rxd),
        .rx_pa(rllllrl_rxa),
        .tx_pd(rllllrl_txd),
        .tx_pa(rllllrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllllrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllllrr_rxr),
        .rx_pd(rllllrr_rxd),
        .rx_pa(rllllrr_rxa),
        .tx_pd(rllllrr_txd),
        .tx_pa(rllllrr_txa)
    );
    PE_left rlllr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h54),
        .rx_pr(rlllr_rxr), .rx_pd(rlllr_rxd), .rx_pa(rlllr_rxa),
        .rx_c0r(rlllrl_rxr), .rx_c0d(rlllrl_rxd), .rx_c0a(rlllrl_rxa),
        .rx_c1r(rlllrr_rxr), .rx_c1d(rlllrr_rxd), .rx_c1a(rlllrr_rxa),
        .tx_c0d(rlllrl_txd), .tx_c0a(rlllrl_txa),
        .tx_c1d(rlllrr_txd), .tx_c1a(rlllrr_txa),
        .tx_pd(rlllr_txd), .tx_pa(rlllr_txa)
    );
    PE_down rlllrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h60),
        .rx_pr(rlllrl_rxr), .rx_pd(rlllrl_rxd), .rx_pa(rlllrl_rxa),
        .rx_c0r(rlllrll_rxr), .rx_c0d(rlllrll_rxd), .rx_c0a(rlllrll_rxa),
        .rx_c1r(rlllrlr_rxr), .rx_c1d(rlllrlr_rxd), .rx_c1a(rlllrlr_rxa),
        .tx_c0d(rlllrll_txd), .tx_c0a(rlllrll_txa),
        .tx_c1d(rlllrlr_txd), .tx_c1a(rlllrlr_txa),
        .tx_pd(rlllrl_txd), .tx_pa(rlllrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlllrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlllrll_rxr),
        .rx_pd(rlllrll_rxd),
        .rx_pa(rlllrll_rxa),
        .tx_pd(rlllrll_txd),
        .tx_pa(rlllrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlllrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlllrlr_rxr),
        .rx_pd(rlllrlr_rxd),
        .rx_pa(rlllrlr_rxa),
        .tx_pd(rlllrlr_txd),
        .tx_pa(rlllrlr_txa)
    );
    PE_up rlllrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h6c),
        .rx_pr(rlllrr_rxr), .rx_pd(rlllrr_rxd), .rx_pa(rlllrr_rxa),
        .rx_c0r(rlllrrl_rxr), .rx_c0d(rlllrrl_rxd), .rx_c0a(rlllrrl_rxa),
        .rx_c1r(rlllrrr_rxr), .rx_c1d(rlllrrr_rxd), .rx_c1a(rlllrrr_rxa),
        .tx_c0d(rlllrrl_txd), .tx_c0a(rlllrrl_txa),
        .tx_c1d(rlllrrr_txd), .tx_c1a(rlllrrr_txa),
        .tx_pd(rlllrr_txd), .tx_pa(rlllrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlllrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlllrrl_rxr),
        .rx_pd(rlllrrl_rxd),
        .rx_pa(rlllrrl_rxa),
        .tx_pd(rlllrrl_txd),
        .tx_pa(rlllrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlllrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlllrrr_rxr),
        .rx_pd(rlllrrr_rxd),
        .rx_pa(rlllrrr_rxa),
        .tx_pd(rlllrrr_txd),
        .tx_pa(rlllrrr_txa)
    );
    PE_up rllr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br1_dat), .cfg_adr(br1_adr), .slv_addr(11'h0),
        .rx_pr(rllr_rxr), .rx_pd(rllr_rxd), .rx_pa(rllr_rxa),
        .rx_c0r(rllrl_rxr), .rx_c0d(rllrl_rxd), .rx_c0a(rllrl_rxa),
        .rx_c1r(rllrr_rxr), .rx_c1d(rllrr_rxd), .rx_c1a(rllrr_rxa),
        .tx_c0d(rllrl_txd), .tx_c0a(rllrl_txa),
        .tx_c1d(rllrr_txd), .tx_c1a(rllrr_txa),
        .tx_pd(rllr_txd), .tx_pa(rllr_txa)
    );
    PE_right rllrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br1_dat), .cfg_adr(br1_adr), .slv_addr(11'hc),
        .rx_pr(rllrl_rxr), .rx_pd(rllrl_rxd), .rx_pa(rllrl_rxa),
        .rx_c0r(rllrll_rxr), .rx_c0d(rllrll_rxd), .rx_c0a(rllrll_rxa),
        .rx_c1r(rllrlr_rxr), .rx_c1d(rllrlr_rxd), .rx_c1a(rllrlr_rxa),
        .tx_c0d(rllrll_txd), .tx_c0a(rllrll_txa),
        .tx_c1d(rllrlr_txd), .tx_c1a(rllrlr_txa),
        .tx_pd(rllrl_txd), .tx_pa(rllrl_txa)
    );
    PE_down rllrll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br1_dat), .cfg_adr(br1_adr), .slv_addr(11'h18),
        .rx_pr(rllrll_rxr), .rx_pd(rllrll_rxd), .rx_pa(rllrll_rxa),
        .rx_c0r(rllrlll_rxr), .rx_c0d(rllrlll_rxd), .rx_c0a(rllrlll_rxa),
        .rx_c1r(rllrllr_rxr), .rx_c1d(rllrllr_rxd), .rx_c1a(rllrllr_rxa),
        .tx_c0d(rllrlll_txd), .tx_c0a(rllrlll_txa),
        .tx_c1d(rllrllr_txd), .tx_c1a(rllrllr_txa),
        .tx_pd(rllrll_txd), .tx_pa(rllrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrlll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrlll_rxr),
        .rx_pd(rllrlll_rxd),
        .rx_pa(rllrlll_rxa),
        .tx_pd(rllrlll_txd),
        .tx_pa(rllrlll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrllr_rxr),
        .rx_pd(rllrllr_rxd),
        .rx_pa(rllrllr_rxa),
        .tx_pd(rllrllr_txd),
        .tx_pa(rllrllr_txa)
    );
    PE_up rllrlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br1_dat), .cfg_adr(br1_adr), .slv_addr(11'h24),
        .rx_pr(rllrlr_rxr), .rx_pd(rllrlr_rxd), .rx_pa(rllrlr_rxa),
        .rx_c0r(rllrlrl_rxr), .rx_c0d(rllrlrl_rxd), .rx_c0a(rllrlrl_rxa),
        .rx_c1r(rllrlrr_rxr), .rx_c1d(rllrlrr_rxd), .rx_c1a(rllrlrr_rxa),
        .tx_c0d(rllrlrl_txd), .tx_c0a(rllrlrl_txa),
        .tx_c1d(rllrlrr_txd), .tx_c1a(rllrlrr_txa),
        .tx_pd(rllrlr_txd), .tx_pa(rllrlr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrlrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrlrl_rxr),
        .rx_pd(rllrlrl_rxd),
        .rx_pa(rllrlrl_rxa),
        .tx_pd(rllrlrl_txd),
        .tx_pa(rllrlrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrlrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrlrr_rxr),
        .rx_pd(rllrlrr_rxd),
        .rx_pa(rllrlrr_rxa),
        .tx_pd(rllrlrr_txd),
        .tx_pa(rllrlrr_txa)
    );
    PE_left rllrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br1_dat), .cfg_adr(br1_adr), .slv_addr(11'h30),
        .rx_pr(rllrr_rxr), .rx_pd(rllrr_rxd), .rx_pa(rllrr_rxa),
        .rx_c0r(rllrrl_rxr), .rx_c0d(rllrrl_rxd), .rx_c0a(rllrrl_rxa),
        .rx_c1r(rllrrr_rxr), .rx_c1d(rllrrr_rxd), .rx_c1a(rllrrr_rxa),
        .tx_c0d(rllrrl_txd), .tx_c0a(rllrrl_txa),
        .tx_c1d(rllrrr_txd), .tx_c1a(rllrrr_txa),
        .tx_pd(rllrr_txd), .tx_pa(rllrr_txa)
    );
    PE_down rllrrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br1_dat), .cfg_adr(br1_adr), .slv_addr(11'h3c),
        .rx_pr(rllrrl_rxr), .rx_pd(rllrrl_rxd), .rx_pa(rllrrl_rxa),
        .rx_c0r(rllrrll_rxr), .rx_c0d(rllrrll_rxd), .rx_c0a(rllrrll_rxa),
        .rx_c1r(rllrrlr_rxr), .rx_c1d(rllrrlr_rxd), .rx_c1a(rllrrlr_rxa),
        .tx_c0d(rllrrll_txd), .tx_c0a(rllrrll_txa),
        .tx_c1d(rllrrlr_txd), .tx_c1a(rllrrlr_txa),
        .tx_pd(rllrrl_txd), .tx_pa(rllrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrrll_rxr),
        .rx_pd(rllrrll_rxd),
        .rx_pa(rllrrll_rxa),
        .tx_pd(rllrrll_txd),
        .tx_pa(rllrrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrrlr_rxr),
        .rx_pd(rllrrlr_rxd),
        .rx_pa(rllrrlr_rxa),
        .tx_pd(rllrrlr_txd),
        .tx_pa(rllrrlr_txa)
    );
    PE_up rllrrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br1_dat), .cfg_adr(br1_adr), .slv_addr(11'h48),
        .rx_pr(rllrrr_rxr), .rx_pd(rllrrr_rxd), .rx_pa(rllrrr_rxa),
        .rx_c0r(rllrrrl_rxr), .rx_c0d(rllrrrl_rxd), .rx_c0a(rllrrrl_rxa),
        .rx_c1r(rllrrrr_rxr), .rx_c1d(rllrrrr_rxd), .rx_c1a(rllrrrr_rxa),
        .tx_c0d(rllrrrl_txd), .tx_c0a(rllrrrl_txa),
        .tx_c1d(rllrrrr_txd), .tx_c1a(rllrrrr_txa),
        .tx_pd(rllrrr_txd), .tx_pa(rllrrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrrrl_rxr),
        .rx_pd(rllrrrl_rxd),
        .rx_pa(rllrrrl_rxa),
        .tx_pd(rllrrrl_txd),
        .tx_pa(rllrrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrrrr_rxr),
        .rx_pd(rllrrrr_rxd),
        .rx_pa(rllrrrr_rxa),
        .tx_pd(rllrrrr_txd),
        .tx_pa(rllrrrr_txa)
    );
    PE_left rlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'h0),
        .rx_pr(rlr_rxr), .rx_pd(rlr_rxd), .rx_pa(rlr_rxa),
        .rx_c0r(rlrl_rxr), .rx_c0d(rlrl_rxd), .rx_c0a(rlrl_rxa),
        .rx_c1r(rlrr_rxr), .rx_c1d(rlrr_rxd), .rx_c1a(rlrr_rxa),
        .tx_c0d(rlrl_txd), .tx_c0a(rlrl_txa),
        .tx_c1d(rlrr_txd), .tx_c1a(rlrr_txa),
        .tx_pd(rlr_txd), .tx_pa(rlr_txa)
    );
    PE_down rlrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'hc),
        .rx_pr(rlrl_rxr), .rx_pd(rlrl_rxd), .rx_pa(rlrl_rxa),
        .rx_c0r(rlrll_rxr), .rx_c0d(rlrll_rxd), .rx_c0a(rlrll_rxa),
        .rx_c1r(rlrlr_rxr), .rx_c1d(rlrlr_rxd), .rx_c1a(rlrlr_rxa),
        .tx_c0d(rlrll_txd), .tx_c0a(rlrll_txa),
        .tx_c1d(rlrlr_txd), .tx_c1a(rlrlr_txa),
        .tx_pd(rlrl_txd), .tx_pa(rlrl_txa)
    );
    PE_right rlrll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'h18),
        .rx_pr(rlrll_rxr), .rx_pd(rlrll_rxd), .rx_pa(rlrll_rxa),
        .rx_c0r(rlrlll_rxr), .rx_c0d(rlrlll_rxd), .rx_c0a(rlrlll_rxa),
        .rx_c1r(rlrllr_rxr), .rx_c1d(rlrllr_rxd), .rx_c1a(rlrllr_rxa),
        .tx_c0d(rlrlll_txd), .tx_c0a(rlrlll_txa),
        .tx_c1d(rlrllr_txd), .tx_c1a(rlrllr_txa),
        .tx_pd(rlrll_txd), .tx_pa(rlrll_txa)
    );
    PE_down rlrlll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'h24),
        .rx_pr(rlrlll_rxr), .rx_pd(rlrlll_rxd), .rx_pa(rlrlll_rxa),
        .rx_c0r(rlrllll_rxr), .rx_c0d(rlrllll_rxd), .rx_c0a(rlrllll_rxa),
        .rx_c1r(rlrlllr_rxr), .rx_c1d(rlrlllr_rxd), .rx_c1a(rlrlllr_rxa),
        .tx_c0d(rlrllll_txd), .tx_c0a(rlrllll_txa),
        .tx_c1d(rlrlllr_txd), .tx_c1a(rlrlllr_txa),
        .tx_pd(rlrlll_txd), .tx_pa(rlrlll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrllll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrllll_rxr),
        .rx_pd(rlrllll_rxd),
        .rx_pa(rlrllll_rxa),
        .tx_pd(rlrllll_txd),
        .tx_pa(rlrllll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrlllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrlllr_rxr),
        .rx_pd(rlrlllr_rxd),
        .rx_pa(rlrlllr_rxa),
        .tx_pd(rlrlllr_txd),
        .tx_pa(rlrlllr_txa)
    );
    PE_up rlrllr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'h30),
        .rx_pr(rlrllr_rxr), .rx_pd(rlrllr_rxd), .rx_pa(rlrllr_rxa),
        .rx_c0r(rlrllrl_rxr), .rx_c0d(rlrllrl_rxd), .rx_c0a(rlrllrl_rxa),
        .rx_c1r(rlrllrr_rxr), .rx_c1d(rlrllrr_rxd), .rx_c1a(rlrllrr_rxa),
        .tx_c0d(rlrllrl_txd), .tx_c0a(rlrllrl_txa),
        .tx_c1d(rlrllrr_txd), .tx_c1a(rlrllrr_txa),
        .tx_pd(rlrllr_txd), .tx_pa(rlrllr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrllrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrllrl_rxr),
        .rx_pd(rlrllrl_rxd),
        .rx_pa(rlrllrl_rxa),
        .tx_pd(rlrllrl_txd),
        .tx_pa(rlrllrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrllrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrllrr_rxr),
        .rx_pd(rlrllrr_rxd),
        .rx_pa(rlrllrr_rxa),
        .tx_pd(rlrllrr_txd),
        .tx_pa(rlrllrr_txa)
    );
    PE_left rlrlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'h3c),
        .rx_pr(rlrlr_rxr), .rx_pd(rlrlr_rxd), .rx_pa(rlrlr_rxa),
        .rx_c0r(rlrlrl_rxr), .rx_c0d(rlrlrl_rxd), .rx_c0a(rlrlrl_rxa),
        .rx_c1r(rlrlrr_rxr), .rx_c1d(rlrlrr_rxd), .rx_c1a(rlrlrr_rxa),
        .tx_c0d(rlrlrl_txd), .tx_c0a(rlrlrl_txa),
        .tx_c1d(rlrlrr_txd), .tx_c1a(rlrlrr_txa),
        .tx_pd(rlrlr_txd), .tx_pa(rlrlr_txa)
    );
    PE_down rlrlrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'h48),
        .rx_pr(rlrlrl_rxr), .rx_pd(rlrlrl_rxd), .rx_pa(rlrlrl_rxa),
        .rx_c0r(rlrlrll_rxr), .rx_c0d(rlrlrll_rxd), .rx_c0a(rlrlrll_rxa),
        .rx_c1r(rlrlrlr_rxr), .rx_c1d(rlrlrlr_rxd), .rx_c1a(rlrlrlr_rxa),
        .tx_c0d(rlrlrll_txd), .tx_c0a(rlrlrll_txa),
        .tx_c1d(rlrlrlr_txd), .tx_c1a(rlrlrlr_txa),
        .tx_pd(rlrlrl_txd), .tx_pa(rlrlrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrlrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrlrll_rxr),
        .rx_pd(rlrlrll_rxd),
        .rx_pa(rlrlrll_rxa),
        .tx_pd(rlrlrll_txd),
        .tx_pa(rlrlrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrlrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrlrlr_rxr),
        .rx_pd(rlrlrlr_rxd),
        .rx_pa(rlrlrlr_rxa),
        .tx_pd(rlrlrlr_txd),
        .tx_pa(rlrlrlr_txa)
    );
    PE_up rlrlrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'h54),
        .rx_pr(rlrlrr_rxr), .rx_pd(rlrlrr_rxd), .rx_pa(rlrlrr_rxa),
        .rx_c0r(rlrlrrl_rxr), .rx_c0d(rlrlrrl_rxd), .rx_c0a(rlrlrrl_rxa),
        .rx_c1r(rlrlrrr_rxr), .rx_c1d(rlrlrrr_rxd), .rx_c1a(rlrlrrr_rxa),
        .tx_c0d(rlrlrrl_txd), .tx_c0a(rlrlrrl_txa),
        .tx_c1d(rlrlrrr_txd), .tx_c1a(rlrlrrr_txa),
        .tx_pd(rlrlrr_txd), .tx_pa(rlrlrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrlrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrlrrl_rxr),
        .rx_pd(rlrlrrl_rxd),
        .rx_pa(rlrlrrl_rxa),
        .tx_pd(rlrlrrl_txd),
        .tx_pa(rlrlrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrlrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrlrrr_rxr),
        .rx_pd(rlrlrrr_rxd),
        .rx_pa(rlrlrrr_rxa),
        .tx_pd(rlrlrrr_txd),
        .tx_pa(rlrlrrr_txa)
    );
    PE_up rlrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br3_dat), .cfg_adr(br3_adr), .slv_addr(11'h0),
        .rx_pr(rlrr_rxr), .rx_pd(rlrr_rxd), .rx_pa(rlrr_rxa),
        .rx_c0r(rlrrl_rxr), .rx_c0d(rlrrl_rxd), .rx_c0a(rlrrl_rxa),
        .rx_c1r(rlrrr_rxr), .rx_c1d(rlrrr_rxd), .rx_c1a(rlrrr_rxa),
        .tx_c0d(rlrrl_txd), .tx_c0a(rlrrl_txa),
        .tx_c1d(rlrrr_txd), .tx_c1a(rlrrr_txa),
        .tx_pd(rlrr_txd), .tx_pa(rlrr_txa)
    );
    PE_right rlrrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br3_dat), .cfg_adr(br3_adr), .slv_addr(11'hc),
        .rx_pr(rlrrl_rxr), .rx_pd(rlrrl_rxd), .rx_pa(rlrrl_rxa),
        .rx_c0r(rlrrll_rxr), .rx_c0d(rlrrll_rxd), .rx_c0a(rlrrll_rxa),
        .rx_c1r(rlrrlr_rxr), .rx_c1d(rlrrlr_rxd), .rx_c1a(rlrrlr_rxa),
        .tx_c0d(rlrrll_txd), .tx_c0a(rlrrll_txa),
        .tx_c1d(rlrrlr_txd), .tx_c1a(rlrrlr_txa),
        .tx_pd(rlrrl_txd), .tx_pa(rlrrl_txa)
    );
    PE_down rlrrll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br3_dat), .cfg_adr(br3_adr), .slv_addr(11'h18),
        .rx_pr(rlrrll_rxr), .rx_pd(rlrrll_rxd), .rx_pa(rlrrll_rxa),
        .rx_c0r(rlrrlll_rxr), .rx_c0d(rlrrlll_rxd), .rx_c0a(rlrrlll_rxa),
        .rx_c1r(rlrrllr_rxr), .rx_c1d(rlrrllr_rxd), .rx_c1a(rlrrllr_rxa),
        .tx_c0d(rlrrlll_txd), .tx_c0a(rlrrlll_txa),
        .tx_c1d(rlrrllr_txd), .tx_c1a(rlrrllr_txa),
        .tx_pd(rlrrll_txd), .tx_pa(rlrrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrlll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrlll_rxr),
        .rx_pd(rlrrlll_rxd),
        .rx_pa(rlrrlll_rxa),
        .tx_pd(rlrrlll_txd),
        .tx_pa(rlrrlll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrllr_rxr),
        .rx_pd(rlrrllr_rxd),
        .rx_pa(rlrrllr_rxa),
        .tx_pd(rlrrllr_txd),
        .tx_pa(rlrrllr_txa)
    );
    PE_up rlrrlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br3_dat), .cfg_adr(br3_adr), .slv_addr(11'h24),
        .rx_pr(rlrrlr_rxr), .rx_pd(rlrrlr_rxd), .rx_pa(rlrrlr_rxa),
        .rx_c0r(rlrrlrl_rxr), .rx_c0d(rlrrlrl_rxd), .rx_c0a(rlrrlrl_rxa),
        .rx_c1r(rlrrlrr_rxr), .rx_c1d(rlrrlrr_rxd), .rx_c1a(rlrrlrr_rxa),
        .tx_c0d(rlrrlrl_txd), .tx_c0a(rlrrlrl_txa),
        .tx_c1d(rlrrlrr_txd), .tx_c1a(rlrrlrr_txa),
        .tx_pd(rlrrlr_txd), .tx_pa(rlrrlr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrlrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrlrl_rxr),
        .rx_pd(rlrrlrl_rxd),
        .rx_pa(rlrrlrl_rxa),
        .tx_pd(rlrrlrl_txd),
        .tx_pa(rlrrlrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrlrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrlrr_rxr),
        .rx_pd(rlrrlrr_rxd),
        .rx_pa(rlrrlrr_rxa),
        .tx_pd(rlrrlrr_txd),
        .tx_pa(rlrrlrr_txa)
    );
    PE_left rlrrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br3_dat), .cfg_adr(br3_adr), .slv_addr(11'h30),
        .rx_pr(rlrrr_rxr), .rx_pd(rlrrr_rxd), .rx_pa(rlrrr_rxa),
        .rx_c0r(rlrrrl_rxr), .rx_c0d(rlrrrl_rxd), .rx_c0a(rlrrrl_rxa),
        .rx_c1r(rlrrrr_rxr), .rx_c1d(rlrrrr_rxd), .rx_c1a(rlrrrr_rxa),
        .tx_c0d(rlrrrl_txd), .tx_c0a(rlrrrl_txa),
        .tx_c1d(rlrrrr_txd), .tx_c1a(rlrrrr_txa),
        .tx_pd(rlrrr_txd), .tx_pa(rlrrr_txa)
    );
    PE_down rlrrrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br3_dat), .cfg_adr(br3_adr), .slv_addr(11'h3c),
        .rx_pr(rlrrrl_rxr), .rx_pd(rlrrrl_rxd), .rx_pa(rlrrrl_rxa),
        .rx_c0r(rlrrrll_rxr), .rx_c0d(rlrrrll_rxd), .rx_c0a(rlrrrll_rxa),
        .rx_c1r(rlrrrlr_rxr), .rx_c1d(rlrrrlr_rxd), .rx_c1a(rlrrrlr_rxa),
        .tx_c0d(rlrrrll_txd), .tx_c0a(rlrrrll_txa),
        .tx_c1d(rlrrrlr_txd), .tx_c1a(rlrrrlr_txa),
        .tx_pd(rlrrrl_txd), .tx_pa(rlrrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrrll_rxr),
        .rx_pd(rlrrrll_rxd),
        .rx_pa(rlrrrll_rxa),
        .tx_pd(rlrrrll_txd),
        .tx_pa(rlrrrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrrlr_rxr),
        .rx_pd(rlrrrlr_rxd),
        .rx_pa(rlrrrlr_rxa),
        .tx_pd(rlrrrlr_txd),
        .tx_pa(rlrrrlr_txa)
    );
    PE_up rlrrrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br3_dat), .cfg_adr(br3_adr), .slv_addr(11'h48),
        .rx_pr(rlrrrr_rxr), .rx_pd(rlrrrr_rxd), .rx_pa(rlrrrr_rxa),
        .rx_c0r(rlrrrrl_rxr), .rx_c0d(rlrrrrl_rxd), .rx_c0a(rlrrrrl_rxa),
        .rx_c1r(rlrrrrr_rxr), .rx_c1d(rlrrrrr_rxd), .rx_c1a(rlrrrrr_rxa),
        .tx_c0d(rlrrrrl_txd), .tx_c0a(rlrrrrl_txa),
        .tx_c1d(rlrrrrr_txd), .tx_c1a(rlrrrrr_txa),
        .tx_pd(rlrrrr_txd), .tx_pa(rlrrrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrrrl_rxr),
        .rx_pd(rlrrrrl_rxd),
        .rx_pa(rlrrrrl_rxa),
        .tx_pd(rlrrrrl_txd),
        .tx_pa(rlrrrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrrrr_rxr),
        .rx_pd(rlrrrrr_rxd),
        .rx_pa(rlrrrrr_rxa),
        .tx_pd(rlrrrrr_txd),
        .tx_pa(rlrrrrr_txa)
    );
    PE_up rr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h0),
        .rx_pr(rr_rxr), .rx_pd(rr_rxd), .rx_pa(rr_rxa),
        .rx_c0r(rrl_rxr), .rx_c0d(rrl_rxd), .rx_c0a(rrl_rxa),
        .rx_c1r(rrr_rxr), .rx_c1d(rrr_rxd), .rx_c1a(rrr_rxa),
        .tx_c0d(rrl_txd), .tx_c0a(rrl_txa),
        .tx_c1d(rrr_txd), .tx_c1a(rrr_txa),
        .tx_pd(rr_txd), .tx_pa(rr_txa)
    );
    PE_right rrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'hc),
        .rx_pr(rrl_rxr), .rx_pd(rrl_rxd), .rx_pa(rrl_rxa),
        .rx_c0r(rrll_rxr), .rx_c0d(rrll_rxd), .rx_c0a(rrll_rxa),
        .rx_c1r(rrlr_rxr), .rx_c1d(rrlr_rxd), .rx_c1a(rrlr_rxa),
        .tx_c0d(rrll_txd), .tx_c0a(rrll_txa),
        .tx_c1d(rrlr_txd), .tx_c1a(rrlr_txa),
        .tx_pd(rrl_txd), .tx_pa(rrl_txa)
    );
    PE_down rrll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h18),
        .rx_pr(rrll_rxr), .rx_pd(rrll_rxd), .rx_pa(rrll_rxa),
        .rx_c0r(rrlll_rxr), .rx_c0d(rrlll_rxd), .rx_c0a(rrlll_rxa),
        .rx_c1r(rrllr_rxr), .rx_c1d(rrllr_rxd), .rx_c1a(rrllr_rxa),
        .tx_c0d(rrlll_txd), .tx_c0a(rrlll_txa),
        .tx_c1d(rrllr_txd), .tx_c1a(rrllr_txa),
        .tx_pd(rrll_txd), .tx_pa(rrll_txa)
    );
    PE_right rrlll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h24),
        .rx_pr(rrlll_rxr), .rx_pd(rrlll_rxd), .rx_pa(rrlll_rxa),
        .rx_c0r(rrllll_rxr), .rx_c0d(rrllll_rxd), .rx_c0a(rrllll_rxa),
        .rx_c1r(rrlllr_rxr), .rx_c1d(rrlllr_rxd), .rx_c1a(rrlllr_rxa),
        .tx_c0d(rrllll_txd), .tx_c0a(rrllll_txa),
        .tx_c1d(rrlllr_txd), .tx_c1a(rrlllr_txa),
        .tx_pd(rrlll_txd), .tx_pa(rrlll_txa)
    );
    PE_down rrllll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h30),
        .rx_pr(rrllll_rxr), .rx_pd(rrllll_rxd), .rx_pa(rrllll_rxa),
        .rx_c0r(rrlllll_rxr), .rx_c0d(rrlllll_rxd), .rx_c0a(rrlllll_rxa),
        .rx_c1r(rrllllr_rxr), .rx_c1d(rrllllr_rxd), .rx_c1a(rrllllr_rxa),
        .tx_c0d(rrlllll_txd), .tx_c0a(rrlllll_txa),
        .tx_c1d(rrllllr_txd), .tx_c1a(rrllllr_txa),
        .tx_pd(rrllll_txd), .tx_pa(rrllll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlllll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlllll_rxr),
        .rx_pd(rrlllll_rxd),
        .rx_pa(rrlllll_rxa),
        .tx_pd(rrlllll_txd),
        .tx_pa(rrlllll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrllllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrllllr_rxr),
        .rx_pd(rrllllr_rxd),
        .rx_pa(rrllllr_rxa),
        .tx_pd(rrllllr_txd),
        .tx_pa(rrllllr_txa)
    );
    PE_up rrlllr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h3c),
        .rx_pr(rrlllr_rxr), .rx_pd(rrlllr_rxd), .rx_pa(rrlllr_rxa),
        .rx_c0r(rrlllrl_rxr), .rx_c0d(rrlllrl_rxd), .rx_c0a(rrlllrl_rxa),
        .rx_c1r(rrlllrr_rxr), .rx_c1d(rrlllrr_rxd), .rx_c1a(rrlllrr_rxa),
        .tx_c0d(rrlllrl_txd), .tx_c0a(rrlllrl_txa),
        .tx_c1d(rrlllrr_txd), .tx_c1a(rrlllrr_txa),
        .tx_pd(rrlllr_txd), .tx_pa(rrlllr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlllrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlllrl_rxr),
        .rx_pd(rrlllrl_rxd),
        .rx_pa(rrlllrl_rxa),
        .tx_pd(rrlllrl_txd),
        .tx_pa(rrlllrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlllrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlllrr_rxr),
        .rx_pd(rrlllrr_rxd),
        .rx_pa(rrlllrr_rxa),
        .tx_pd(rrlllrr_txd),
        .tx_pa(rrlllrr_txa)
    );
    PE_left rrllr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h48),
        .rx_pr(rrllr_rxr), .rx_pd(rrllr_rxd), .rx_pa(rrllr_rxa),
        .rx_c0r(rrllrl_rxr), .rx_c0d(rrllrl_rxd), .rx_c0a(rrllrl_rxa),
        .rx_c1r(rrllrr_rxr), .rx_c1d(rrllrr_rxd), .rx_c1a(rrllrr_rxa),
        .tx_c0d(rrllrl_txd), .tx_c0a(rrllrl_txa),
        .tx_c1d(rrllrr_txd), .tx_c1a(rrllrr_txa),
        .tx_pd(rrllr_txd), .tx_pa(rrllr_txa)
    );
    PE_down rrllrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h54),
        .rx_pr(rrllrl_rxr), .rx_pd(rrllrl_rxd), .rx_pa(rrllrl_rxa),
        .rx_c0r(rrllrll_rxr), .rx_c0d(rrllrll_rxd), .rx_c0a(rrllrll_rxa),
        .rx_c1r(rrllrlr_rxr), .rx_c1d(rrllrlr_rxd), .rx_c1a(rrllrlr_rxa),
        .tx_c0d(rrllrll_txd), .tx_c0a(rrllrll_txa),
        .tx_c1d(rrllrlr_txd), .tx_c1a(rrllrlr_txa),
        .tx_pd(rrllrl_txd), .tx_pa(rrllrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrllrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrllrll_rxr),
        .rx_pd(rrllrll_rxd),
        .rx_pa(rrllrll_rxa),
        .tx_pd(rrllrll_txd),
        .tx_pa(rrllrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrllrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrllrlr_rxr),
        .rx_pd(rrllrlr_rxd),
        .rx_pa(rrllrlr_rxa),
        .tx_pd(rrllrlr_txd),
        .tx_pa(rrllrlr_txa)
    );
    PE_up rrllrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h60),
        .rx_pr(rrllrr_rxr), .rx_pd(rrllrr_rxd), .rx_pa(rrllrr_rxa),
        .rx_c0r(rrllrrl_rxr), .rx_c0d(rrllrrl_rxd), .rx_c0a(rrllrrl_rxa),
        .rx_c1r(rrllrrr_rxr), .rx_c1d(rrllrrr_rxd), .rx_c1a(rrllrrr_rxa),
        .tx_c0d(rrllrrl_txd), .tx_c0a(rrllrrl_txa),
        .tx_c1d(rrllrrr_txd), .tx_c1a(rrllrrr_txa),
        .tx_pd(rrllrr_txd), .tx_pa(rrllrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrllrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrllrrl_rxr),
        .rx_pd(rrllrrl_rxd),
        .rx_pa(rrllrrl_rxa),
        .tx_pd(rrllrrl_txd),
        .tx_pa(rrllrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrllrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrllrrr_rxr),
        .rx_pd(rrllrrr_rxd),
        .rx_pa(rrllrrr_rxa),
        .tx_pd(rrllrrr_txd),
        .tx_pa(rrllrrr_txa)
    );
    PE_up rrlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br5_dat), .cfg_adr(br5_adr), .slv_addr(11'h0),
        .rx_pr(rrlr_rxr), .rx_pd(rrlr_rxd), .rx_pa(rrlr_rxa),
        .rx_c0r(rrlrl_rxr), .rx_c0d(rrlrl_rxd), .rx_c0a(rrlrl_rxa),
        .rx_c1r(rrlrr_rxr), .rx_c1d(rrlrr_rxd), .rx_c1a(rrlrr_rxa),
        .tx_c0d(rrlrl_txd), .tx_c0a(rrlrl_txa),
        .tx_c1d(rrlrr_txd), .tx_c1a(rrlrr_txa),
        .tx_pd(rrlr_txd), .tx_pa(rrlr_txa)
    );
    PE_right rrlrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br5_dat), .cfg_adr(br5_adr), .slv_addr(11'hc),
        .rx_pr(rrlrl_rxr), .rx_pd(rrlrl_rxd), .rx_pa(rrlrl_rxa),
        .rx_c0r(rrlrll_rxr), .rx_c0d(rrlrll_rxd), .rx_c0a(rrlrll_rxa),
        .rx_c1r(rrlrlr_rxr), .rx_c1d(rrlrlr_rxd), .rx_c1a(rrlrlr_rxa),
        .tx_c0d(rrlrll_txd), .tx_c0a(rrlrll_txa),
        .tx_c1d(rrlrlr_txd), .tx_c1a(rrlrlr_txa),
        .tx_pd(rrlrl_txd), .tx_pa(rrlrl_txa)
    );
    PE_down rrlrll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br5_dat), .cfg_adr(br5_adr), .slv_addr(11'h18),
        .rx_pr(rrlrll_rxr), .rx_pd(rrlrll_rxd), .rx_pa(rrlrll_rxa),
        .rx_c0r(rrlrlll_rxr), .rx_c0d(rrlrlll_rxd), .rx_c0a(rrlrlll_rxa),
        .rx_c1r(rrlrllr_rxr), .rx_c1d(rrlrllr_rxd), .rx_c1a(rrlrllr_rxa),
        .tx_c0d(rrlrlll_txd), .tx_c0a(rrlrlll_txa),
        .tx_c1d(rrlrllr_txd), .tx_c1a(rrlrllr_txa),
        .tx_pd(rrlrll_txd), .tx_pa(rrlrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrlll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrlll_rxr),
        .rx_pd(rrlrlll_rxd),
        .rx_pa(rrlrlll_rxa),
        .tx_pd(rrlrlll_txd),
        .tx_pa(rrlrlll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrllr_rxr),
        .rx_pd(rrlrllr_rxd),
        .rx_pa(rrlrllr_rxa),
        .tx_pd(rrlrllr_txd),
        .tx_pa(rrlrllr_txa)
    );
    PE_up rrlrlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br5_dat), .cfg_adr(br5_adr), .slv_addr(11'h24),
        .rx_pr(rrlrlr_rxr), .rx_pd(rrlrlr_rxd), .rx_pa(rrlrlr_rxa),
        .rx_c0r(rrlrlrl_rxr), .rx_c0d(rrlrlrl_rxd), .rx_c0a(rrlrlrl_rxa),
        .rx_c1r(rrlrlrr_rxr), .rx_c1d(rrlrlrr_rxd), .rx_c1a(rrlrlrr_rxa),
        .tx_c0d(rrlrlrl_txd), .tx_c0a(rrlrlrl_txa),
        .tx_c1d(rrlrlrr_txd), .tx_c1a(rrlrlrr_txa),
        .tx_pd(rrlrlr_txd), .tx_pa(rrlrlr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrlrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrlrl_rxr),
        .rx_pd(rrlrlrl_rxd),
        .rx_pa(rrlrlrl_rxa),
        .tx_pd(rrlrlrl_txd),
        .tx_pa(rrlrlrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrlrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrlrr_rxr),
        .rx_pd(rrlrlrr_rxd),
        .rx_pa(rrlrlrr_rxa),
        .tx_pd(rrlrlrr_txd),
        .tx_pa(rrlrlrr_txa)
    );
    PE_left rrlrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br5_dat), .cfg_adr(br5_adr), .slv_addr(11'h30),
        .rx_pr(rrlrr_rxr), .rx_pd(rrlrr_rxd), .rx_pa(rrlrr_rxa),
        .rx_c0r(rrlrrl_rxr), .rx_c0d(rrlrrl_rxd), .rx_c0a(rrlrrl_rxa),
        .rx_c1r(rrlrrr_rxr), .rx_c1d(rrlrrr_rxd), .rx_c1a(rrlrrr_rxa),
        .tx_c0d(rrlrrl_txd), .tx_c0a(rrlrrl_txa),
        .tx_c1d(rrlrrr_txd), .tx_c1a(rrlrrr_txa),
        .tx_pd(rrlrr_txd), .tx_pa(rrlrr_txa)
    );
    PE_down rrlrrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br5_dat), .cfg_adr(br5_adr), .slv_addr(11'h3c),
        .rx_pr(rrlrrl_rxr), .rx_pd(rrlrrl_rxd), .rx_pa(rrlrrl_rxa),
        .rx_c0r(rrlrrll_rxr), .rx_c0d(rrlrrll_rxd), .rx_c0a(rrlrrll_rxa),
        .rx_c1r(rrlrrlr_rxr), .rx_c1d(rrlrrlr_rxd), .rx_c1a(rrlrrlr_rxa),
        .tx_c0d(rrlrrll_txd), .tx_c0a(rrlrrll_txa),
        .tx_c1d(rrlrrlr_txd), .tx_c1a(rrlrrlr_txa),
        .tx_pd(rrlrrl_txd), .tx_pa(rrlrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrrll_rxr),
        .rx_pd(rrlrrll_rxd),
        .rx_pa(rrlrrll_rxa),
        .tx_pd(rrlrrll_txd),
        .tx_pa(rrlrrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrrlr_rxr),
        .rx_pd(rrlrrlr_rxd),
        .rx_pa(rrlrrlr_rxa),
        .tx_pd(rrlrrlr_txd),
        .tx_pa(rrlrrlr_txa)
    );
    PE_up rrlrrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br5_dat), .cfg_adr(br5_adr), .slv_addr(11'h48),
        .rx_pr(rrlrrr_rxr), .rx_pd(rrlrrr_rxd), .rx_pa(rrlrrr_rxa),
        .rx_c0r(rrlrrrl_rxr), .rx_c0d(rrlrrrl_rxd), .rx_c0a(rrlrrrl_rxa),
        .rx_c1r(rrlrrrr_rxr), .rx_c1d(rrlrrrr_rxd), .rx_c1a(rrlrrrr_rxa),
        .tx_c0d(rrlrrrl_txd), .tx_c0a(rrlrrrl_txa),
        .tx_c1d(rrlrrrr_txd), .tx_c1a(rrlrrrr_txa),
        .tx_pd(rrlrrr_txd), .tx_pa(rrlrrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrrrl_rxr),
        .rx_pd(rrlrrrl_rxd),
        .rx_pa(rrlrrrl_rxa),
        .tx_pd(rrlrrrl_txd),
        .tx_pa(rrlrrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrrrr_rxr),
        .rx_pd(rrlrrrr_rxd),
        .rx_pa(rrlrrrr_rxa),
        .tx_pd(rrlrrrr_txd),
        .tx_pa(rrlrrrr_txa)
    );
    PE_left rrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'h0),
        .rx_pr(rrr_rxr), .rx_pd(rrr_rxd), .rx_pa(rrr_rxa),
        .rx_c0r(rrrl_rxr), .rx_c0d(rrrl_rxd), .rx_c0a(rrrl_rxa),
        .rx_c1r(rrrr_rxr), .rx_c1d(rrrr_rxd), .rx_c1a(rrrr_rxa),
        .tx_c0d(rrrl_txd), .tx_c0a(rrrl_txa),
        .tx_c1d(rrrr_txd), .tx_c1a(rrrr_txa),
        .tx_pd(rrr_txd), .tx_pa(rrr_txa)
    );
    PE_down rrrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'hc),
        .rx_pr(rrrl_rxr), .rx_pd(rrrl_rxd), .rx_pa(rrrl_rxa),
        .rx_c0r(rrrll_rxr), .rx_c0d(rrrll_rxd), .rx_c0a(rrrll_rxa),
        .rx_c1r(rrrlr_rxr), .rx_c1d(rrrlr_rxd), .rx_c1a(rrrlr_rxa),
        .tx_c0d(rrrll_txd), .tx_c0a(rrrll_txa),
        .tx_c1d(rrrlr_txd), .tx_c1a(rrrlr_txa),
        .tx_pd(rrrl_txd), .tx_pa(rrrl_txa)
    );
    PE_right rrrll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'h18),
        .rx_pr(rrrll_rxr), .rx_pd(rrrll_rxd), .rx_pa(rrrll_rxa),
        .rx_c0r(rrrlll_rxr), .rx_c0d(rrrlll_rxd), .rx_c0a(rrrlll_rxa),
        .rx_c1r(rrrllr_rxr), .rx_c1d(rrrllr_rxd), .rx_c1a(rrrllr_rxa),
        .tx_c0d(rrrlll_txd), .tx_c0a(rrrlll_txa),
        .tx_c1d(rrrllr_txd), .tx_c1a(rrrllr_txa),
        .tx_pd(rrrll_txd), .tx_pa(rrrll_txa)
    );
    PE_down rrrlll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'h24),
        .rx_pr(rrrlll_rxr), .rx_pd(rrrlll_rxd), .rx_pa(rrrlll_rxa),
        .rx_c0r(rrrllll_rxr), .rx_c0d(rrrllll_rxd), .rx_c0a(rrrllll_rxa),
        .rx_c1r(rrrlllr_rxr), .rx_c1d(rrrlllr_rxd), .rx_c1a(rrrlllr_rxa),
        .tx_c0d(rrrllll_txd), .tx_c0a(rrrllll_txa),
        .tx_c1d(rrrlllr_txd), .tx_c1a(rrrlllr_txa),
        .tx_pd(rrrlll_txd), .tx_pa(rrrlll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrllll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrllll_rxr),
        .rx_pd(rrrllll_rxd),
        .rx_pa(rrrllll_rxa),
        .tx_pd(rrrllll_txd),
        .tx_pa(rrrllll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrlllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrlllr_rxr),
        .rx_pd(rrrlllr_rxd),
        .rx_pa(rrrlllr_rxa),
        .tx_pd(rrrlllr_txd),
        .tx_pa(rrrlllr_txa)
    );
    PE_up rrrllr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'h30),
        .rx_pr(rrrllr_rxr), .rx_pd(rrrllr_rxd), .rx_pa(rrrllr_rxa),
        .rx_c0r(rrrllrl_rxr), .rx_c0d(rrrllrl_rxd), .rx_c0a(rrrllrl_rxa),
        .rx_c1r(rrrllrr_rxr), .rx_c1d(rrrllrr_rxd), .rx_c1a(rrrllrr_rxa),
        .tx_c0d(rrrllrl_txd), .tx_c0a(rrrllrl_txa),
        .tx_c1d(rrrllrr_txd), .tx_c1a(rrrllrr_txa),
        .tx_pd(rrrllr_txd), .tx_pa(rrrllr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrllrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrllrl_rxr),
        .rx_pd(rrrllrl_rxd),
        .rx_pa(rrrllrl_rxa),
        .tx_pd(rrrllrl_txd),
        .tx_pa(rrrllrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrllrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrllrr_rxr),
        .rx_pd(rrrllrr_rxd),
        .rx_pa(rrrllrr_rxa),
        .tx_pd(rrrllrr_txd),
        .tx_pa(rrrllrr_txa)
    );
    PE_left rrrlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'h3c),
        .rx_pr(rrrlr_rxr), .rx_pd(rrrlr_rxd), .rx_pa(rrrlr_rxa),
        .rx_c0r(rrrlrl_rxr), .rx_c0d(rrrlrl_rxd), .rx_c0a(rrrlrl_rxa),
        .rx_c1r(rrrlrr_rxr), .rx_c1d(rrrlrr_rxd), .rx_c1a(rrrlrr_rxa),
        .tx_c0d(rrrlrl_txd), .tx_c0a(rrrlrl_txa),
        .tx_c1d(rrrlrr_txd), .tx_c1a(rrrlrr_txa),
        .tx_pd(rrrlr_txd), .tx_pa(rrrlr_txa)
    );
    PE_down rrrlrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'h48),
        .rx_pr(rrrlrl_rxr), .rx_pd(rrrlrl_rxd), .rx_pa(rrrlrl_rxa),
        .rx_c0r(rrrlrll_rxr), .rx_c0d(rrrlrll_rxd), .rx_c0a(rrrlrll_rxa),
        .rx_c1r(rrrlrlr_rxr), .rx_c1d(rrrlrlr_rxd), .rx_c1a(rrrlrlr_rxa),
        .tx_c0d(rrrlrll_txd), .tx_c0a(rrrlrll_txa),
        .tx_c1d(rrrlrlr_txd), .tx_c1a(rrrlrlr_txa),
        .tx_pd(rrrlrl_txd), .tx_pa(rrrlrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrlrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrlrll_rxr),
        .rx_pd(rrrlrll_rxd),
        .rx_pa(rrrlrll_rxa),
        .tx_pd(rrrlrll_txd),
        .tx_pa(rrrlrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrlrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrlrlr_rxr),
        .rx_pd(rrrlrlr_rxd),
        .rx_pa(rrrlrlr_rxa),
        .tx_pd(rrrlrlr_txd),
        .tx_pa(rrrlrlr_txa)
    );
    PE_up rrrlrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'h54),
        .rx_pr(rrrlrr_rxr), .rx_pd(rrrlrr_rxd), .rx_pa(rrrlrr_rxa),
        .rx_c0r(rrrlrrl_rxr), .rx_c0d(rrrlrrl_rxd), .rx_c0a(rrrlrrl_rxa),
        .rx_c1r(rrrlrrr_rxr), .rx_c1d(rrrlrrr_rxd), .rx_c1a(rrrlrrr_rxa),
        .tx_c0d(rrrlrrl_txd), .tx_c0a(rrrlrrl_txa),
        .tx_c1d(rrrlrrr_txd), .tx_c1a(rrrlrrr_txa),
        .tx_pd(rrrlrr_txd), .tx_pa(rrrlrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrlrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrlrrl_rxr),
        .rx_pd(rrrlrrl_rxd),
        .rx_pa(rrrlrrl_rxa),
        .tx_pd(rrrlrrl_txd),
        .tx_pa(rrrlrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrlrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrlrrr_rxr),
        .rx_pd(rrrlrrr_rxd),
        .rx_pa(rrrlrrr_rxa),
        .tx_pd(rrrlrrr_txd),
        .tx_pa(rrrlrrr_txa)
    );
    PE_up rrrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br7_dat), .cfg_adr(br7_adr), .slv_addr(11'h0),
        .rx_pr(rrrr_rxr), .rx_pd(rrrr_rxd), .rx_pa(rrrr_rxa),
        .rx_c0r(rrrrl_rxr), .rx_c0d(rrrrl_rxd), .rx_c0a(rrrrl_rxa),
        .rx_c1r(rrrrr_rxr), .rx_c1d(rrrrr_rxd), .rx_c1a(rrrrr_rxa),
        .tx_c0d(rrrrl_txd), .tx_c0a(rrrrl_txa),
        .tx_c1d(rrrrr_txd), .tx_c1a(rrrrr_txa),
        .tx_pd(rrrr_txd), .tx_pa(rrrr_txa)
    );
    PE_right rrrrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br7_dat), .cfg_adr(br7_adr), .slv_addr(11'hc),
        .rx_pr(rrrrl_rxr), .rx_pd(rrrrl_rxd), .rx_pa(rrrrl_rxa),
        .rx_c0r(rrrrll_rxr), .rx_c0d(rrrrll_rxd), .rx_c0a(rrrrll_rxa),
        .rx_c1r(rrrrlr_rxr), .rx_c1d(rrrrlr_rxd), .rx_c1a(rrrrlr_rxa),
        .tx_c0d(rrrrll_txd), .tx_c0a(rrrrll_txa),
        .tx_c1d(rrrrlr_txd), .tx_c1a(rrrrlr_txa),
        .tx_pd(rrrrl_txd), .tx_pa(rrrrl_txa)
    );
    PE_down rrrrll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br7_dat), .cfg_adr(br7_adr), .slv_addr(11'h18),
        .rx_pr(rrrrll_rxr), .rx_pd(rrrrll_rxd), .rx_pa(rrrrll_rxa),
        .rx_c0r(rrrrlll_rxr), .rx_c0d(rrrrlll_rxd), .rx_c0a(rrrrlll_rxa),
        .rx_c1r(rrrrllr_rxr), .rx_c1d(rrrrllr_rxd), .rx_c1a(rrrrllr_rxa),
        .tx_c0d(rrrrlll_txd), .tx_c0a(rrrrlll_txa),
        .tx_c1d(rrrrllr_txd), .tx_c1a(rrrrllr_txa),
        .tx_pd(rrrrll_txd), .tx_pa(rrrrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrlll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrlll_rxr),
        .rx_pd(rrrrlll_rxd),
        .rx_pa(rrrrlll_rxa),
        .tx_pd(rrrrlll_txd),
        .tx_pa(rrrrlll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrllr_rxr),
        .rx_pd(rrrrllr_rxd),
        .rx_pa(rrrrllr_rxa),
        .tx_pd(rrrrllr_txd),
        .tx_pa(rrrrllr_txa)
    );
    PE_up rrrrlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br7_dat), .cfg_adr(br7_adr), .slv_addr(11'h24),
        .rx_pr(rrrrlr_rxr), .rx_pd(rrrrlr_rxd), .rx_pa(rrrrlr_rxa),
        .rx_c0r(rrrrlrl_rxr), .rx_c0d(rrrrlrl_rxd), .rx_c0a(rrrrlrl_rxa),
        .rx_c1r(rrrrlrr_rxr), .rx_c1d(rrrrlrr_rxd), .rx_c1a(rrrrlrr_rxa),
        .tx_c0d(rrrrlrl_txd), .tx_c0a(rrrrlrl_txa),
        .tx_c1d(rrrrlrr_txd), .tx_c1a(rrrrlrr_txa),
        .tx_pd(rrrrlr_txd), .tx_pa(rrrrlr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrlrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrlrl_rxr),
        .rx_pd(rrrrlrl_rxd),
        .rx_pa(rrrrlrl_rxa),
        .tx_pd(rrrrlrl_txd),
        .tx_pa(rrrrlrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrlrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrlrr_rxr),
        .rx_pd(rrrrlrr_rxd),
        .rx_pa(rrrrlrr_rxa),
        .tx_pd(rrrrlrr_txd),
        .tx_pa(rrrrlrr_txa)
    );
    PE_left rrrrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br7_dat), .cfg_adr(br7_adr), .slv_addr(11'h30),
        .rx_pr(rrrrr_rxr), .rx_pd(rrrrr_rxd), .rx_pa(rrrrr_rxa),
        .rx_c0r(rrrrrl_rxr), .rx_c0d(rrrrrl_rxd), .rx_c0a(rrrrrl_rxa),
        .rx_c1r(rrrrrr_rxr), .rx_c1d(rrrrrr_rxd), .rx_c1a(rrrrrr_rxa),
        .tx_c0d(rrrrrl_txd), .tx_c0a(rrrrrl_txa),
        .tx_c1d(rrrrrr_txd), .tx_c1a(rrrrrr_txa),
        .tx_pd(rrrrr_txd), .tx_pa(rrrrr_txa)
    );
    PE_down rrrrrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br7_dat), .cfg_adr(br7_adr), .slv_addr(11'h3c),
        .rx_pr(rrrrrl_rxr), .rx_pd(rrrrrl_rxd), .rx_pa(rrrrrl_rxa),
        .rx_c0r(rrrrrll_rxr), .rx_c0d(rrrrrll_rxd), .rx_c0a(rrrrrll_rxa),
        .rx_c1r(rrrrrlr_rxr), .rx_c1d(rrrrrlr_rxd), .rx_c1a(rrrrrlr_rxa),
        .tx_c0d(rrrrrll_txd), .tx_c0a(rrrrrll_txa),
        .tx_c1d(rrrrrlr_txd), .tx_c1a(rrrrrlr_txa),
        .tx_pd(rrrrrl_txd), .tx_pa(rrrrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrrll_rxr),
        .rx_pd(rrrrrll_rxd),
        .rx_pa(rrrrrll_rxa),
        .tx_pd(rrrrrll_txd),
        .tx_pa(rrrrrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrrlr_rxr),
        .rx_pd(rrrrrlr_rxd),
        .rx_pa(rrrrrlr_rxa),
        .tx_pd(rrrrrlr_txd),
        .tx_pa(rrrrrlr_txa)
    );
    PE_up rrrrrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br7_dat), .cfg_adr(br7_adr), .slv_addr(11'h48),
        .rx_pr(rrrrrr_rxr), .rx_pd(rrrrrr_rxd), .rx_pa(rrrrrr_rxa),
        .rx_c0r(rrrrrrl_rxr), .rx_c0d(rrrrrrl_rxd), .rx_c0a(rrrrrrl_rxa),
        .rx_c1r(rrrrrrr_rxr), .rx_c1d(rrrrrrr_rxd), .rx_c1a(rrrrrrr_rxa),
        .tx_c0d(rrrrrrl_txd), .tx_c0a(rrrrrrl_txa),
        .tx_c1d(rrrrrrr_txd), .tx_c1a(rrrrrrr_txa),
        .tx_pd(rrrrrr_txd), .tx_pa(rrrrrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrrrl_rxr),
        .rx_pd(rrrrrrl_rxd),
        .rx_pa(rrrrrrl_rxa),
        .tx_pd(rrrrrrl_txd),
        .tx_pa(rrrrrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrrrr_rxr),
        .rx_pd(rrrrrrr_rxd),
        .rx_pa(rrrrrrr_rxa),
        .tx_pd(rrrrrrr_txd),
        .tx_pa(rrrrrrr_txa)
    );
endmodule
`default_nettype none
`define MPRJ_IO_PADS 38

module user_proj_example #(
    parameter BITS = 32
) (
`ifdef USE_POWER_PINS
    inout vccd1,    // User area 1 1.8V supply
    inout vssd1,    // User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wire wb_clk_i,
    input wire wb_rst_i,
    input wire wbs_stb_i,
    input wire wbs_cyc_i,
    input wire wbs_we_i,
    input wire [3:0] wbs_sel_i,
    input wire [31:0] wbs_dat_i,
    input wire [31:0] wbs_adr_i,
    output wire wbs_ack_o,
    output wire [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  wire [127:0] la_data_in,
    output wire [127:0] la_data_out,
    input  wire [127:0] la_oenb,

    // IOs
    input  wire [`MPRJ_IO_PADS-1:0] io_in,
    output wire [`MPRJ_IO_PADS-1:0] io_out,
    output wire [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    inout wire [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input wire user_clock2,

    // User maskable interrupt signals
    output wire [2:0] user_irq
);

  Top i_Top (
    .wb_clk_i (wb_clk_i ),
    .wb_rst_i (wb_rst_i ),
    .wbs_stb_i(wbs_stb_i),
    .wbs_cyc_i(wbs_cyc_i),
    .wbs_we_i (wbs_we_i ),
    .wbs_sel_i(wbs_sel_i),
    .wbs_dat_i(wbs_dat_i),
    .wbs_adr_i(wbs_adr_i),
    .wbs_ack_o(wbs_ack_o),
    .wbs_dat_o(wbs_dat_o)
  );

endmodule

`default_nettype wire
