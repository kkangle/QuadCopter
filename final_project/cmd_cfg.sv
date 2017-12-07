module cmd_cfg(clk,rst_n,cmd_rdy,cmd,batt,data,cal_done,cnv_cmplt,clr_cmd_rdy,resp,send_resp,d_ptch,d_roll,d_yaw,thrst,strt_cal,inertial_cal,motors_off,strt_cnv);
//Yudong Chen
	input clk,rst_n;
	input cmd_rdy;
	input [7:0] cmd,batt;
	input [15:0] data;
	input cal_done;
	input cnv_cmplt;

	output logic clr_cmd_rdy;
	output logic [7:0] resp;
	output logic send_resp;
	output logic signed[15:0] d_ptch,d_roll,d_yaw;
	output logic [8:0]thrst;//unsigned!!!!!!!!!!!!!!!
	output logic strt_cal;
	output logic inertial_cal;
	output logic motors_off;
	output logic strt_cnv;
	
	logic wptch,wroll,wyaw,wthrst,mtrs_off,en_mtrs,clr_tmr,emergency;

	parameter mtr_ramp_tmr = 5'd26;
	logic [mtr_ramp_tmr-1:0]motor_cnt;
	
	always@(posedge clk, negedge rst_n)begin
		if(rst_n==0)begin
			d_ptch<=$signed(0);
		end
		else if (emergency==1)begin
			d_ptch<=$signed(0);
		end
		else if (wptch==1)begin
			d_ptch<=$signed(data);//data is signed or unsigned???
		end
			//else  hold
	end
	
	always@(posedge clk, negedge rst_n)begin
		if(rst_n==0)begin
			d_roll<=$signed(0);
		end
		else if (emergency==1)begin
			d_roll<=$signed(0);
		end
		else if (wroll==1)begin
			d_roll<=$signed(data);
		end
			//else  hold
	end
	
	always@(posedge clk, negedge rst_n)begin
		if(rst_n==0)begin
			d_yaw<=$signed(0);
		end
		else if (emergency==1)begin
			d_yaw<=$signed(0);
		end
		else if (wyaw==1)begin
			d_yaw<=$signed(data);
		end
			//else  hold
	end
	
	always@(posedge clk, negedge rst_n)begin
		if(rst_n==0)begin
			thrst<=0;
		end
		else if (emergency==1)begin
			thrst<=0;
		end
		else if (wthrst==1)begin
			thrst<=data[8:0];
		end
			//else  hold
	end
		
	always@(posedge clk, negedge rst_n)begin
		if(rst_n==0)begin
			motors_off<=1;//preset!!!!!!!!!!!!!!!
		end
		else if (mtrs_off==1)begin
			motors_off<=1;
		end
		else if (en_mtrs==1)begin
			motors_off<=0;
		end
			//else hold 
		
	end
		
	always@(posedge clk)begin
		if (clr_tmr==1)begin
			motor_cnt<=0;
		end
		else begin
			motor_cnt<=motor_cnt+1;
		end
	end
		
		
	typedef enum reg[2:0] {IDLE,SEND_ACK,CAL1,CAL2,READ_BATT}state_t;
	state_t state,nextstate;
	
	typedef enum reg[3:0] {no_use,REQ_BATT,SET_PTCH,SET_ROLL,SET_YAW,SET_THRST,CALIBRATE,EMER_LAND,MTRS_OFF}cmd_type;//1-8 in use 

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
		clr_cmd_rdy=0;
		strt_cnv=0;
		wptch=0;
		wroll=0;
		wyaw=0;
		wthrst=0;
		clr_tmr=0;
		en_mtrs=0;
		emergency=0;
		mtrs_off=0;
		send_resp=0;
		resp=0;
		strt_cal=0;
		inertial_cal=0;
		
		case(state)
			IDLE:begin
				nextstate=IDLE;
				if(cmd_rdy==1)begin
					clr_cmd_rdy=1;
					case(cmd)
						REQ_BATT:begin
							nextstate=READ_BATT;
							strt_cnv=1;
						end
						SET_PTCH:begin
							wptch=1;
							nextstate=SEND_ACK;
						end
						SET_ROLL:begin
							wroll=1;
							nextstate=SEND_ACK;
						end
						SET_YAW:begin
							wyaw=1;
							nextstate=SEND_ACK;
						end
						SET_THRST:begin
							wthrst=1;
							nextstate=SEND_ACK;
						end
							
						CALIBRATE:begin//??
							clr_tmr=1;
							en_mtrs=1;
							nextstate=CAL1;
						end
						EMER_LAND:begin
							emergency=1;
							nextstate=SEND_ACK;
						end
						
						MTRS_OFF:begin//until??
							mtrs_off=1;
							nextstate=SEND_ACK;
						end
						default:begin
							
						end

					endcase
					
					
					
				end
			
			end
			
			SEND_ACK:begin
				resp=8'hA5;
				send_resp=1;
				nextstate=IDLE;
			end
				
			READ_BATT:begin
				if(cnv_cmplt==1)begin
					resp=batt;
					send_resp=1;//?true?
					nextstate=IDLE;
				end
				else begin
					nextstate=READ_BATT;
				end
			end
			CAL1:begin
				if(&motor_cnt==1)begin//don't need tmr_full, dont care   26'h3FE56C0
					nextstate=CAL2;
					strt_cal=1;
					inertial_cal=1;
				end
				else begin
					
					nextstate=CAL1;
				end
			end
			CAL2:begin
				if(cal_done==1)begin
					nextstate=SEND_ACK;
				end
				else begin
					inertial_cal=1;
					nextstate=CAL2;
				end
			end
			default:begin
				nextstate=IDLE;
			end
		
		endcase
	end

	

endmodule