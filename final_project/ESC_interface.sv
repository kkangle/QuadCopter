module ESC_interface(clk,rst_n,SPEED,OFF,PWM);

input clk,rst_n;
input [10:0] SPEED;
input [9:0] OFF;

output logic PWM;


parameter PERIOD_WIDTH=20;
logic [PERIOD_WIDTH-1:0]cnt;
wire [11:0]compensated_speed;
wire [16:0]speed_setting;
wire Rst,Set;

assign compensated_speed=SPEED+OFF;
assign speed_setting=(compensated_speed<<4)+16'd50000;
assign Rst=cnt>=speed_setting?1:0;//high time pass , reset//// do we need to limit to lower digit
assign Set=(&cnt)?1:0;//time cycle end high

always_ff@(posedge clk,negedge rst_n)begin//sequential,non
	if(rst_n==0)begin
		PWM<=0;
	end
	else if(Set)begin
		PWM<=1;
	end
	else if(Rst)begin
		PWM<=0;
	end
	else begin
		PWM<=PWM;
	end
end


//50mhz cycle
always_ff@(posedge clk,negedge rst_n)begin//sequencial,non
	if(rst_n==0)begin
		cnt<=0;
	end
	else begin
		cnt<=cnt+1;
	end
end


endmodule 
