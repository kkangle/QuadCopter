module commaster(clk, rst_n, cmd, snd_cmd, data, TX, RX, frm_snt, rx_data, rx_rdy);

input clk,rst_n,snd_cmd, RX;
input [7:0] cmd;
input [15:0] data;
output TX, frm_snt, rx_rdy;
output [7:0] rx_data;
logic [1:0] sel;
logic frm_snt, tx_done, rx_rdy, clr_rx_rdy, trmt, tx_dong, set_cmplt, clr_cmplt;

reg [7:0] flop1;
reg [7:0] flop2 ;
reg [7:0] tx_data, rx_data;

UART uart_trasiver(.clk(clk),.rst_n(rst_n),.TX(TX),.RX(RX),.rx_rdy(rx_rdy),.clr_rx_rdy(clr_rx_rdy),
					.rx_data(rx_data),.trmt(trmt),.tx_data(tx_data),.tx_done(tx_done));
typedef enum reg[1:0] {idle, waith, waitm, waitl} state_t;
state_t state, nxt_state; 

// flop for upper 8bits data
always@(posedge clk, negedge rst_n) begin
	if(!rst_n)
		flop1 <= 8'h00;
	else if(snd_cmd)
		flop1 <= data[15:8];
	else
		flop2 <= flop1;
end

// flop for lower 8bits data
always@(posedge clk, negedge rst_n) begin

	if(!rst_n)
		flop2 <= 8'h00;
	else if(snd_cmd)
		flop2 <= data[7:0];
	else 
		flop2 <= flop2;
end

// flop for frm_snt output 
always@(posedge clk, negedge rst_n) begin
	if(!rst_n)
		frm_snt <= 0;
	else if(clr_cmplt)
		frm_snt <= 0;
	else if(set_cmplt)
		frm_snt <= 1;
	else 
		frm_snt <= 0;
end

// txdata select
assign tx_data = (sel == 2'b10)? cmd:
				 (sel == 2'b01)? flop1:
				 flop2;
				
				  
			  
always@(posedge clk, negedge rst_n)begin 
	if(!rst_n)
		state <= idle;
	else
		state <= nxt_state;
end



always_comb begin
	trmt = 0;
	set_cmplt = 0;
	clr_cmplt = 0;
	nxt_state = idle;
	sel = 2'b10;
	case(state)
		idle: begin
			if(snd_cmd) begin
				trmt = 1;
				clr_cmplt = 1;
				nxt_state = waith;
				sel = 2'b10;
			end
			else
				nxt_state = idle;
		end
		
		waith: begin
			if(tx_done) begin
				sel = 2'b01;
				trmt = 1;
				nxt_state = waitm;
			end
			else 
				 nxt_state = waith;
				 
		end
			
		waitm: begin 
			if(tx_done) begin
				sel = 2'b00;
				trmt = 1;
				nxt_state = waitl;
			end
			else
				nxt_state = waitm;
		end
		
		waitl: begin 
			if(tx_done) begin
				set_cmplt = 1;
				nxt_state = idle;
			end
			else
				nxt_state = waitl;
		end
	endcase
end



endmodule