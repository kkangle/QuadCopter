
module CommMaster(snd_cmd,data,cmd,clk,rst_n,RX,TX,frm_snt,clr_frm_snt_out,rx_data,rx_rdy);
//yudong chen 
	input snd_cmd;
	input [15:0]data;
	input [7:0] cmd;
	input clk, rst_n;
	input RX;
	input clr_frm_snt_out;
	
	output logic  TX;
	output logic  frm_snt;
	output logic  rx_rdy;
	output logic  [7:0]rx_data;
	
	logic [7:0]tx_data;
	logic tx_done;
	logic [7:0]highdata,lowdata;
	logic trmt;
	logic [1:0]sel;
	logic set_frm_snt,clr_frm_snt;
	
	
	UART iUART(.clk(clk),.rst_n(rst_n),.RX(RX),.TX(TX),.rx_rdy(rx_rdy),.clr_rx_rdy(),.rx_data(rx_data),.trmt(trmt),.tx_data(tx_data),.tx_done(tx_done));

	
	always@(posedge clk)begin
		if(snd_cmd==1)begin
			highdata<=data[15:8];
			lowdata<=data[7:0];
		end
			//else hold 
	end
	
	
	always_comb begin
		if(sel[1]==0 && sel[0]==0)begin
			tx_data=cmd;
		end
		else if (sel[1]==0 && sel[0]==1)begin
			tx_data =highdata;
		end
		else begin
			tx_data=lowdata;
		end

	end
	
	
	
	always@(posedge clk,negedge rst_n)begin
		if(rst_n==0)begin
			frm_snt<=0;
		end
		else if (set_frm_snt==1)begin
			frm_snt<=1;
		end
		else if (clr_frm_snt==1)begin
			frm_snt<=0;
		end
		else if (clr_frm_snt_out==1)begin
			frm_snt<=0;
		end
			//else hold 
	end
		
//-----------------sm--------------//
	typedef enum reg[1:0] {IDLE,CMD,HIGH,LOW}state_t;
	state_t state,nextstate;

	//state transition 
	always_ff@(posedge clk,negedge rst_n)begin
		if(rst_n==0)begin
			state<=IDLE;
		end
		else begin
			state<=nextstate;
		end
	end

		
	always_comb begin
		sel=2'b00;
		nextstate=IDLE;
		trmt=0;
		clr_frm_snt=0;
		set_frm_snt=0;
		case(state)
		
			IDLE:begin
				if(snd_cmd==1)begin
					$display("idle to cmd");
					sel=2'b00;
					nextstate=CMD;
					trmt = 1;
					clr_frm_snt=1;
				end
			
			end
			CMD:begin
			    sel = 2'b01;//////////////////////////////////////////////////////////////good 
				if(tx_done==1)begin
					nextstate=HIGH;
					trmt=1;
				end
				else begin
					$display("cmd");
					nextstate=CMD;
				end
			end
			HIGH:begin
				sel=2'b10;
				if(tx_done==1)begin
					trmt=1;
					nextstate=LOW;
				end
				else begin
					$display("high");
					nextstate=HIGH;
				end

			end
			LOW:begin
				if(tx_done==1)begin
					//sel=2'b00;
					nextstate=IDLE;
					set_frm_snt=1;
				end
				else begin
					//sel=2'b10;
					nextstate=LOW;
				end
			
			end
			
			default:begin
				nextstate=IDLE;
			end
		endcase
	
	end
		
	
	
	
	

endmodule 
	