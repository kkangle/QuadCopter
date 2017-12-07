module A2D_intf(clk,rst_n,strt_cnv,chnnl,cnv_cmplt,res,SS_n,SCLK,MOSI,MISO);
	input clk, rst_n;
	input strt_cnv;
	input [2:0]chnnl;
	input MISO;
	output logic  cnv_cmplt;
	output logic [11:0] res;
	output logic SS_n,SCLK,MOSI;
	logic done,wrt,finish;
	logic [15:0]rd_data;
	logic [15:0]cmd;

	SPI_mstr16 ispi(.clk(clk),.rst_n(rst_n),.wrt(wrt),.cmd(cmd),.MISO(MISO),.done(done),.rd_data(rd_data),.SS_n(SS_n),.SCLK(SCLK),.MOSI(MOSI));


	always@(posedge clk,negedge rst_n)begin
		if(rst_n==0)begin
			cnv_cmplt=0;
		end
		else if (strt_cnv==1)begin
			cnv_cmplt=0;
		end
		else if (finish==1)begin
			cnv_cmplt=1;
		end
			//else hold 
	end
		
	assign res=rd_data[11:0];
	assign cmd={2'b00,chnnl,11'h000};
//---------------sm-----------------------
	typedef enum reg[1:0] {IDLE,SEND1,WAIT,SEND2}state_t;
	state_t state,nextstate;

	always_ff@(posedge clk,negedge rst_n)begin
		if(rst_n==0)begin
			state<=IDLE;
		end
		else begin
			state<=nextstate;
		end
	end

	always_comb begin
		nextstate=IDLE;
		wrt=0;
		finish=0;
		case(state)
			IDLE:begin
				if(strt_cnv==1)begin
					nextstate=SEND1;
					wrt=1;				
				end
			end
			SEND1:begin
				if(done==1)begin
					nextstate=WAIT;
				end
				else begin
					nextstate=SEND1;
				end
			end
			WAIT:begin
				nextstate=SEND2;
				wrt=1;
			end
			SEND2:begin
				if(done==1)begin
					nextstate=IDLE;
					finish=1;
				end
				else begin
					nextstate=SEND2;
				end
			end
			default:begin
				nextstate=IDLE;
			end
		endcase

	end

endmodule