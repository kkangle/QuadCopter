module flght_cntrl(clk,rst_n,vld,inertial_cal,d_ptch,d_roll,d_yaw,ptch,
					roll,yaw,thrst,frnt_spd,bck_spd,lft_spd,rght_spd);
				
parameter D_QUEUE_DEPTH = 14;		// delay for derivative term
				
input clk,rst_n;
input vld;									// tells when a new valid inertial reading ready
											// only update D_QUEUE on vld readings
input inertial_cal;							// need to run motors at CAL_SPEED during inertial calibration
input signed [15:0] d_ptch,d_roll,d_yaw;	// desired pitch roll and yaw (from cmd_cfg)
input signed [15:0] ptch,roll,yaw;			// actual pitch roll and yaw (from inertial interface)
input [8:0] thrst;							// thrust level from slider
output [10:0] frnt_spd;						// 11-bit unsigned speed at which to run front motor
output [10:0] bck_spd;						// 11-bit unsigned speed at which to back front motor
output [10:0] lft_spd;						// 11-bit unsigned speed at which to left front motor
output [10:0] rght_spd;						// 11-bit unsigned speed at which to right front motor

///////////////////////////////////////////////////
// Need integer for loop used to create D_QUEUE //
/////////////////////////////////////////////////
integer x;
//////////////////////////////
// Define needed registers //
////////////////////////////								
reg signed [9:0] prev_ptch_err[0:D_QUEUE_DEPTH-1];
reg signed [9:0] prev_roll_err[0:D_QUEUE_DEPTH-1];
reg signed [9:0] prev_yaw_err[0:D_QUEUE_DEPTH-1];	// need previous error terms for D of PD

//////////////////////////////////////////////////////
// You will need a bunch of interal wires declared //
// for intermediate math results...do that here   //
///////////////////////////////////////////////////
wire signed[16:0]ptch_err,roll_err,yaw_err;
wire signed[9:0]ptch_err_sat,roll_err_sat,yaw_err_sat;
wire signed[9:0]ptch_pterm,roll_pterm,yaw_pterm;
wire signed[9:0]ptch_D_diff,roll_D_diff,yaw_D_diff;
wire signed[5:0]ptch_D_diff_sat,roll_D_diff_sat,yaw_D_diff_sat;
wire signed[11:0]ptch_dterm,roll_dterm,yaw_dterm;
wire signed[12:0]frnt_calc,bck_calc, lft_calc, rght_calc;
wire [10:0]frnt_calc_sat,bck_calc_sat, lft_calc_sat, rght_calc_sat;
///////////////////////////////////////////////////////////////
// some Parameters to keep things more generic and flexible //
/////////////////////////////////////////////////////////////
  
  localparam CAL_SPEED = 11'h1B0;		// speed to run motors at during inertial calibration
  localparam MIN_RUN_SPEED = 13'h200;	// minimum speed while running  
  localparam D_COEFF = 6'b00111;			// D coefficient in PID control = +7
  
  
/// OK...rest is up to you...good luck! ///
 //Form error term of actual angle minus desired angle(i.e. ptch – d_ptch), make it 17-bits wide so it can’t overflow.
assign ptch_err=ptch-d_ptch;
assign roll_err=roll-d_roll;
assign yaw_err=yaw-d_yaw;

//Saturate the error term to 10-bits, call it: *_err_sat (i.e. ptch_err_sat)
assign ptch_err_sat = (~ptch_err[16] && |ptch_err[15:9]) ? 10'h1FF : // sat pos
 (ptch_err[16] && ~&ptch_err[15:9]) ? 10'h200 : // sat neg
ptch_err[9:0]; 
assign roll_err_sat = (~roll_err[16] && |roll_err[15:9]) ? 10'h1FF : // sat pos
 (roll_err[16] && ~&roll_err[15:9]) ? 10'h200 : // sat neg
roll_err[9:0]; 
assign yaw_err_sat = (~yaw_err[16] && |yaw_err[15:9]) ? 10'h1FF : // sat pos
 (yaw_err[16] && ~&yaw_err[15:9]) ? 10'h200 : // sat neg
yaw_err[9:0]; 




//Create *_pterm which is 5/8 of the saturated error term.
assign ptch_pterm={ptch_err_sat[9],ptch_err_sat[9:1]}+{{3{ptch_err_sat[9]}},ptch_err_sat[9:3]};
assign roll_pterm={roll_err_sat[9],roll_err_sat[9:1]}+{{3{roll_err_sat[9]}},roll_err_sat[9:3]};
assign yaw_pterm={yaw_err_sat[9],yaw_err_sat[9:1]}+{{3{yaw_err_sat[9]}},yaw_err_sat[9:3]};

//Create a queue of parametized depth and width of 10-bits. 
always_ff @(posedge clk, negedge rst_n)begin
	if(rst_n==0)begin
		for(x=0;x<D_QUEUE_DEPTH;x=x+1)begin
			prev_ptch_err[x]=10'h000;
			prev_roll_err[x]=10'h000;
			prev_yaw_err[x]=10'h000;
		end
	end
	else if (vld)begin
		for(x=D_QUEUE_DEPTH-1;x>0;x=x-1)begin
			prev_ptch_err[x]=prev_ptch_err[x-1];
			prev_roll_err[x]=prev_roll_err[x-1];
			prev_yaw_err[x]=prev_yaw_err[x-1];
		end
		prev_ptch_err[0]=ptch_err_sat;
		prev_roll_err[0]=roll_err_sat;
		prev_yaw_err[0]=yaw_err_sat;
	end

end
//Form *_D_diff which is the current saturated error minus the oldest queue entry. Could this overflow a 10-bit value? Not with any reasonable values we could possibly encounter during flight, so just keep it a 10-bit value.
assign ptch_D_diff=ptch_err_sat-prev_ptch_err[D_QUEUE_DEPTH-1];
assign roll_D_diff=roll_err_sat-prev_roll_err[D_QUEUE_DEPTH-1];
assign yaw_D_diff=yaw_err_sat-prev_yaw_err[D_QUEUE_DEPTH-1];

//Saturate *_D_diff to 6-bits in width (max pos value of 0x1F, max neg of 0x20)
assign ptch_D_diff_sat = (~ptch_D_diff[9] && |ptch_D_diff[9:5]) ? 6'b01_1111 : // sat pos
 (ptch_D_diff[9] && ~&ptch_D_diff[9:5]) ? 6'b10_0000 : // sat neg
ptch_D_diff[5:0];
assign roll_D_diff_sat = (~roll_D_diff[9] && |roll_D_diff[9:5]) ? 6'b01_1111 : // sat pos
 (roll_D_diff[9] && ~&roll_D_diff[9:5]) ? 6'b10_0000 : // sat neg
roll_D_diff[5:0];
assign yaw_D_diff_sat = (~yaw_D_diff[9] && |yaw_D_diff[9:5]) ? 6'b01_1111 : // sat pos
 (yaw_D_diff[9] && ~&yaw_D_diff[9:5]) ? 6'b10_0000 : // sat neg
yaw_D_diff[5:0];
//Multiply the saturated *_D_diff 6-bit value by a 6-bit constant defined in a localparam. This should be a signed multiplier and will give a 12-bit product.
assign ptch_dterm=ptch_D_diff_sat*$signed(D_COEFF);
assign roll_dterm=roll_D_diff_sat*$signed(D_COEFF);
assign yaw_dterm=yaw_D_diff_sat*$signed(D_COEFF);

//Sign extend and add/subtract the various terms to form the various calculated motor speeds. I called these intermediate terms frnt_calc, bck_calc, lft_calc, rght_calc.
assign frnt_calc=$signed({4'b0000,thrst[8:0]})+$signed(MIN_RUN_SPEED)-{{3{ptch_pterm[9]}},ptch_pterm[9:0]}-{{1{ptch_dterm[11]}},ptch_dterm[11:0]}-{{3{yaw_pterm[9]}},yaw_pterm[9:0]}-{{1{yaw_dterm[11]}},yaw_dterm[11:0]};
assign bck_calc=$signed({4'b0000,thrst[8:0]})+$signed(MIN_RUN_SPEED)+{{3{ptch_pterm[9]}},ptch_pterm[9:0]}+{{1{ptch_dterm[11]}},ptch_dterm[11:0]}-{{3{yaw_pterm[9]}},yaw_pterm[9:0]}-{{1{yaw_dterm[11]}},yaw_dterm[11:0]};

assign lft_calc=$signed({4'b0000,thrst[8:0]})+$signed(MIN_RUN_SPEED)-{{3{roll_pterm[9]}},roll_pterm[9:0]}-{{1{roll_dterm[11]}},roll_dterm[11:0]}+{{3{yaw_pterm[9]}},yaw_pterm[9:0]}+{{1{yaw_dterm[11]}},yaw_dterm[11:0]};

assign rght_calc=$signed({4'b0000,thrst[8:0]})+$signed(MIN_RUN_SPEED)+{{3{roll_pterm[9]}},roll_pterm[9:0]}+{{1{roll_dterm[11]}},roll_dterm[11:0]}+{{3{yaw_pterm[9]}},yaw_pterm[9:0]}+{{1{yaw_dterm[11]}},yaw_dterm[11:0]};

//Saturate these 13-bit values to 11-bits. This is an unsigned saturation. Motors on a quadcopter can’t be driven backwards. Best you can do is stop the motor, so if the value is less than zero then clamp it at zero. If it is greater than 0x7FF then clamp it at 0x7FF
assign frnt_calc_sat = (~frnt_calc[12] && |frnt_calc[12:11]) ? 11'b111_1111_1111 : // sat pos
 (frnt_calc[12]) ? 11'b0 : // sat neg
frnt_calc[10:0];
assign bck_calc_sat = (~bck_calc[12] && |bck_calc[12:11]) ? 11'b111_1111_1111 : // sat pos
 (bck_calc[12]) ? 11'b0 : // sat neg
bck_calc[10:0];
assign lft_calc_sat = (~lft_calc[12] && |lft_calc[12:11]) ? 11'b111_1111_1111 : // sat pos
 (lft_calc[12]) ? 11'b0: // sat neg
lft_calc[10:0];

assign rght_calc_sat = (~rght_calc[12] && |rght_calc[12:11]) ? 11'b111_1111_1111 : // sat pos
 (rght_calc[12]) ? 11'b0: // sat neg
rght_calc[10:0];

//Finally infer the mux to force the motor speed to CAL_SPEED during calibration. CAL_SPEED will be an 11-bit constant defined via a localparam.
assign frnt_spd=inertial_cal? CAL_SPEED:frnt_calc_sat;
assign bck_spd=inertial_cal? CAL_SPEED:bck_calc_sat;
assign lft_spd=inertial_cal? CAL_SPEED:lft_calc_sat;
assign rght_spd=inertial_cal? CAL_SPEED:rght_calc_sat;

  
endmodule 
