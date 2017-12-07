module UART_wrapper(clk, rst_n, clr_cmd_rdy, snd_resp,resp,cmd_rdy, resp_sent,cmd,data,RX,TX);

input clr_cmd_rdy, snd_resp;
input clk, rst_n;
input [7:0] resp;
output logic cmd_rdy, resp_sent;
output logic [7:0] cmd;
output logic [15:0] data;

input RX;
output logic TX;

logic [7:0] rx_data;
logic set_cmd_rdy, clr_cmd_rdy_i, data_load, cmd_load;
logic [7:0] data_high;
logic trmt, clr_rdy, rdy;

UART iUART(.clk(clk),.rst_n(rst_n),.RX(RX),.TX(TX),.rx_rdy(rdy),.clr_rx_rdy(clr_rdy),.rx_data(rx_data),.trmt(snd_resp),.tx_data(resp),.tx_done(resp_sent));



assign data = {data_high,rx_data};

always@(posedge clk) begin

	if(!rst_n) cmd <= 7'b0000000;
	else if(cmd_load) cmd <= rx_data;

end

always@(posedge clk) begin

	if(!rst_n) data_high <= 8'h00;
	else if(data_load) data_high <= rx_data;

end


always@(posedge clk, negedge rst_n) begin

	if(!rst_n) cmd_rdy <= 1'b0;
	else if(clr_cmd_rdy) cmd_rdy <= 1'b0;
	else if(clr_cmd_rdy_i) cmd_rdy <= 1'b0;
	else if(set_cmd_rdy) cmd_rdy <= 1'b1;

end


typedef enum reg [1:0] {IDLE,CMD,DATA} state_t;
state_t state, next_state;

always@(posedge clk,negedge rst_n) begin 
	if(!rst_n) state <= IDLE;
	else state <= next_state;
end

always_comb begin 
	next_state = IDLE;
	clr_cmd_rdy_i = 0;
	cmd_load = 0;
	clr_rdy = 0;
	set_cmd_rdy = 0;
	data_load = 0;
	case(state)

		IDLE: begin
			if(rdy) begin
				next_state = CMD;
				clr_cmd_rdy_i = 1;
				cmd_load = 1;
				clr_rdy = 1;
			end
		end

		CMD: begin
			if(rdy) begin
				next_state = DATA;
				data_load = 1;
				clr_rdy = 1;
			end
			else begin
				next_state=CMD;
			end
		end

		DATA: begin
			if(rdy) begin
				next_state = IDLE;
				set_cmd_rdy = 1;
				clr_rdy = 1;
			end
			else begin
				next_state=DATA;
			end
		end
		
		default: next_state = IDLE;
	
	endcase
end


endmodule

