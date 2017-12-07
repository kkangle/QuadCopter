//v2.0 2017年11月20日17:45:47 
module SPI_mstr16(clk,rst_n,wrt,cmd,MISO,done,rd_data,SS_n,SCLK,MOSI);
	input clk,rst_n;//50MHz system clock and reset
	input wrt;//A high for 1 clock period would initiate a SPI transaction
	input [15:0]cmd;//Data (command) being sent to inertial sensor or A2D converter.
	input MISO;

	output logic done;//Asserted when SPI transaction is complete. Should stay asserted till next wrt
	output logic [15:0]rd_data;
	output logic SS_n;
	output logic SCLK;
	output logic MOSI;
	
	logic smpl;
	logic shft;
	logic rst_cnt;
	logic transmitting;
	logic [3:0]bit_cnt;
	logic [4:0]sclk_div;

	logic MISO_smpl;
	logic [15:0]shft_reg;
	logic set_ssn,clr_ssn;
	logic set_done,clr_done;
		
	
	//sclk and its use 
	always@(posedge clk)begin
		if(rst_cnt==1)begin
			sclk_div<=5'b10111;
		end
		else if(transmitting==1)begin
			sclk_div<=sclk_div+1;
		end
			//else remain
	end
	assign SCLK=sclk_div[4];
	//sample miso 
	always@(posedge clk)begin
		if(smpl==1)begin
			MISO_smpl<=MISO;
		end
		//else maintain
	end
	//shift reg...
	always@(posedge clk)begin
		if(wrt==1)begin
			shft_reg<=cmd;
		end
		else if(shft==1)begin
			shft_reg<={shft_reg[14:0],MISO_smpl};
		end
		//else maintain
	end
	//bit count max 16
	always@(posedge clk)begin
		if(rst_cnt==1)begin
			bit_cnt<=0;
		end
		else if (shft==1)begin
			bit_cnt<=bit_cnt+1;
		end
		//else remain
	end
	//assign output 
	assign MOSI=shft_reg[15];
		
	always@(posedge clk,negedge rst_n)begin
		if(rst_n==0)begin
			SS_n<=0;
		end
		else if(set_ssn==1)begin
			SS_n<=1;
		end
		else if(clr_ssn==1)begin
			SS_n<=0;
		end
			//else hold 
	end
	always@(posedge clk,negedge rst_n)begin
		if(rst_n==0)begin
			done<=0;
		end
		else if(set_done==1)begin
			done<=1;
		end
		else if(clr_done==1)begin
			done<=0;
		end
			//else hold 
	end

	//assign rd_data=(transmitting)?16'hxxxx:shft_reg;
	assign rd_data=shft_reg;
	
		typedef enum reg[1:0] {IDLE,FRONTPORCH,TRMT,BACKPORCH}state_t;
	state_t state,nextstate;

	always_ff@(posedge clk,negedge rst_n)begin
		if(rst_n==0)begin
			state<=IDLE;
		end
		else begin
			state<=nextstate;
		end
	end

	always_comb begin//initial avoid latch
		nextstate=IDLE;
		rst_cnt=0;
		smpl=0;
		shft=0;
		transmitting=0;
		set_done=0;
		clr_done=0;
		set_ssn=0;
		clr_ssn=0;
		case(state)
		IDLE:begin
			if(wrt==1)begin//start
				clr_done=1;
				clr_ssn=1;
				nextstate=FRONTPORCH;
				rst_cnt=1;
				transmitting=1;
			end
				//else remain idle
		end
		
		FRONTPORCH:begin//wait till front 
			if(sclk_div==5'b11111)begin
				nextstate=TRMT;
				transmitting=1;
			end
			else begin
				nextstate=FRONTPORCH;	
				transmitting=1;
			end
		end
		TRMT:begin
			if(sclk_div==5'b01111)begin//sample
				smpl=1;
				transmitting=1;
				nextstate=TRMT;
			end
			else if(sclk_div==5'b11111)begin//shift
				if(bit_cnt==15)begin//last bit 
					nextstate=IDLE;
					shft=1;
					transmitting=0;
					nextstate=IDLE;//new 
					set_done=1;
					set_ssn=1;
				end
				else begin//not last bit 
					shft=1;
					nextstate=TRMT;
					transmitting=1;
				end
			end
			else begin//wait until finish
				nextstate=TRMT;
				transmitting=1;
			end
		end
		
		
		// BACKPORCH:begin
			// nextstate=IDLE;
			
		// end
		default:begin
			nextstate=IDLE;
		end	
		endcase
	end





endmodule
