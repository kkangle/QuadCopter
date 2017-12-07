module rst_synch(RST_n,clk,rst_n);

input RST_n,clk;
output logic rst_n;
logic ff1_out;

//ff1 and ff2
always@(negedge clk,negedge RST_n)begin//sequential non
	if(RST_n==0)begin
		ff1_out<=0;
		rst_n<=0;
	end
	else begin
		ff1_out<=1;
		rst_n<=ff1_out;
	end
	
end



endmodule 
