`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,

    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  reg  [(pDATA_WIDTH-1):0] rdata,   

    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    input   wire                     sm_tready, 
    output  reg                      sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  reg  [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

// FSM
parameter S_IDLE  = 3'd0;
parameter S_DATA  = 3'd1;
parameter S_CAL   = 3'd2;
parameter S_DONE = 3'd3;

reg [1:0] current_state, next_state;

reg [(pADDR_WIDTH-1):0] raddr, waddr;
reg [(pDATA_WIDTH-1):0] write_data;
reg [(pDATA_WIDTH-1):0] ss_tdata_received;
reg got_waddr, got_wdata, got_raddr, got_rdata, calculating, first_data, last_data;
wire can_write;

reg ap_start_flag, ap_start, ap_done, ap_idle;
wire [(pDATA_WIDTH-1):0] configuration;

reg [5:0] addr_cnt;
wire [5:0] addr_cnt_next;
reg [1:0] cal_cnt;
reg [(pDATA_WIDTH-1):0] mult_in1, add_in2, add_out;
wire [(pDATA_WIDTH-1):0] mult_in2, mult_out;

// FSM
always @(*) begin
    case (current_state)
        S_IDLE: begin
            if (ap_start) next_state = S_DATA;
            else
                next_state = S_IDLE;
        end
        S_DATA: begin
            if (ss_tvalid)
                next_state = S_CAL;
            else
                next_state = S_DATA;
        end
        S_CAL: begin
            if (sm_tlast)
                next_state = S_IDLE;
            else
                next_state = S_CAL;
        end
        default:
            next_state = S_IDLE; 
    endcase
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n)    current_state <= S_IDLE;
    else                current_state <= next_state;
end

// axi-lite read address
assign arready = !rvalid;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        raddr <= 0;
    end
    else begin
        if (arvalid && arready) 
            raddr <= araddr;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        got_raddr <= 0;
    end
    else begin
        if (arvalid && arready)
            got_raddr <= 1;
        else
            got_raddr <= 0;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        got_rdata <= 0;
    end
    else begin
        if (got_raddr)
            got_rdata <= 1;
        else
            got_rdata <= 0;
    end
end

// axi-lite read data
assign rvalid = got_rdata;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        rdata <= 0;
    end
    else begin
        if(got_raddr && raddr == 12'h000)
            rdata <= configuration;
        else if(got_raddr && raddr >= 12'h020) 
            rdata <= tap_Do;
    end
end

// axi-lite write address
assign awready = !got_waddr;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        waddr <= 0;
    end
    else begin
        if (awvalid && awready) 
            waddr <= awaddr;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        got_waddr <= 0;
    end
    else begin
        if (awvalid && awready)
            got_waddr <= 1;
        else if (got_wdata)
            got_waddr <= 0;
    end
end

// axi-lite write data
assign wready = !got_wdata;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        write_data <= 0;
    end
    else begin
        if (wvalid && wready) 
            write_data <= wdata;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        got_wdata <= 0;
    end
    else begin
        if (wvalid && wready) 
            got_wdata <= 1;
        else
            got_wdata <= 0;
    end
end

assign configuration = {29'b0, ap_idle, ap_done, ap_start};

// ap_start
// always @(posedge axis_clk or negedge axis_rst_n) begin
//     if (!axis_rst_n) begin
//         ap_start_flag <= 0;
//     end
//     else begin
//         if (can_write && waddr == 12'h000) 
//             ap_start_flag <= 1;
//         else if (ap_done)
//             ap_start_flag <= 0;
//     end
// end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        ap_start <= 0;
    end
    else begin
        if (can_write && waddr == 12'h000 && ap_idle && !ap_done) 
            ap_start <= 1;
        else
            ap_start <= 0;
    end
end

// ap_done
always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        ap_done <= 0;
    end
    else begin
        if (sm_tready && sm_tvalid && sm_tlast) 
            ap_done <= 1;
        else if (raddr == 12'h000 && rvalid && rdata[1] == 1)
            ap_done <= 0;
    end
end

// ap_idle
always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        ap_idle <= 1;
    end
    else begin 
        if(sm_tready && sm_tvalid && sm_tlast) 
            ap_idle <= 1;    
        else if(ap_start)      
            ap_idle <= 0;
    end

    // else begin
    //     if (wvalid && wready) 
    //         got_wdata <= 1;
    //     else
    //         got_wdata <= 0;
    // end
end

// tap bram
assign can_write = got_waddr && got_wdata;
assign tap_EN = 1;
assign tap_WE = {4{can_write && waddr != 12'h000 && waddr != 12'h010}};
// assign tap_Di = wdata;
assign tap_Di = write_data;
always @(*) begin
    if (can_write && waddr >= 12'h020)
        tap_A = waddr - 12'h020;
    // else if (got_raddr)
    else if(arready && arvalid && araddr != 12'h000 && araddr != 12'h010)
        tap_A = araddr - 12'h020;
    else
        tap_A = addr_cnt;
end

// data bram
// assign can_write_
assign data_EN = 1;
assign data_WE = {4{cal_cnt == 1}};
assign data_A = (cal_cnt == 0 && addr_cnt != 0) ? addr_cnt - 4 : addr_cnt;
assign data_Di = mult_in1;
assign addr_cnt_next = addr_cnt - 4;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        addr_cnt <= 40;
    end
    else begin
        if (cal_cnt == 2) 
            addr_cnt <= addr_cnt_next;
        else if (!calculating)
            addr_cnt <= 40;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        cal_cnt <= 0;
    end
    else begin
        if (calculating) 
            cal_cnt <= (cal_cnt == 2) ? 0 : cal_cnt + 1;
        else
            cal_cnt <= 0;
    end
end

// calculation
always @(*) begin
    if (addr_cnt == 0)
        mult_in1 = ss_tdata_received;
    else if (first_data)
        mult_in1 = 0;
    else
        mult_in1 = data_Do;
end
assign mult_in2 = tap_Do;
assign mult_out = mult_in1 * mult_in2;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        add_in2 <= 0;
    end
    else begin
        add_in2 <= mult_out;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        calculating <= 0;
    end
    else begin
        if (ss_tvalid && ss_tready) 
            calculating <= 1;
        else if (sm_tvalid)
            calculating <= 0;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)begin
        add_out <= 0;
    end
    else begin
        if(sm_tready && sm_tvalid) 
            add_out <= 0;
        else if(calculating && cal_cnt == 2) 
            add_out <= add_out + add_in2;
    end
end

 always@(posedge axis_clk or negedge axis_rst_n)begin
    if (!axis_rst_n)begin
        first_data <= 1;
    end
    else begin
        if(sm_tready && sm_tvalid) 
            first_data <= 0;
        else if (ap_start)
            first_data <= 1;
    end
end

// ss
assign ss_tready = current_state == S_CAL && !calculating;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        ss_tdata_received <= 0;
    end
    else begin
        if (ss_tvalid && ss_tready) 
            ss_tdata_received <= ss_tdata;
    end
end

// sm
always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        sm_tvalid <= 0;
    end
    else begin
        if (addr_cnt == 0 && cal_cnt == 2) 
            sm_tvalid <= 1;
        else if (sm_tvalid)
            sm_tvalid <= 0;
    end
end

// always @(posedge axis_clk or negedge axis_rst_n) begin
//     if (!axis_rst_n) begin
//         sm_tdata <= 0;
//     end
//     else begin
//         if (addr_cnt == 0 && cal_cnt == 2) 
//             sm_tdata <= add_out;
//         else if (sm_tvalid)
//             sm_tdata <= 0;
//     end
// end
assign sm_tdata = (addr_cnt == 60 && cal_cnt == 0) ? add_out : 0;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        last_data <= 0;
    end
    else begin
        if (ss_tlast && sm_tvalid) 
            last_data <= 1;
        else if (ap_start)
            last_data <= 0;
    end
end

assign sm_tlast = last_data && sm_tvalid;

// always @(posedge axis_clk or negedge axis_rst_n) begin
//     if (!axis_rst_n) begin
//         sm_tlast <= 0;
//     end
//     else begin
//         if (ss_tvalid && ss_tready) 
//             ss_tdata_received <= ss_tdata;
//     end
// end

endmodule