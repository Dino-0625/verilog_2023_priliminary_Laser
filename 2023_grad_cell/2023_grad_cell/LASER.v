module LASER (
input CLK,
input RST,
input [3:0] X,
input [3:0] Y,
output reg [3:0] C1X,
output reg [3:0] C1Y,
output reg [3:0] C2X,
output reg [3:0] C2Y,
output reg DONE);

parameter GETDATA = 1;
parameter ITERATION = 2;
parameter STOP = 3;
parameter FINISH = 4;
parameter MASKING = 5;
parameter UNMASKING = 6;
reg item [0:255];
reg [7:0] position, positionRecord;
reg [4:0] count;
reg [2:0] state,nextState;
reg [5:0] target1sum, target2sum, sum, totalsum;

reg finishGetdata, finishIteration, firstUNMask, positionChange, targetChoose, finishMasking, finishUNMasking, gotoUNMasking, allStop;
wire n64, n50, n49, n48, n47, n46, n35, n34, n33, n32, n31, n30, n29, n19, n18, n17, n16, n15, n14, n13, n4, n3, n2, n1;
wire center, p64, p50, p49, p48, p47, p46, p35, p34, p33, p32, p31, p30, p29, p19, p18, p17, p16, p15, p14, p13, p4, p3, p2, p1;
wire [3:0] level1, level2, level3, level4, level5, level6, level7, level8, level9;

wire [5:0] sum4x4;
reg maskcenter;
reg maskp [0:23];
reg maskn [0:23];


assign level1 = (position[7:4] >= 4) ? n64 : 0;
assign level2 = (position[7:4] >= 3) ? (n50 + n49 + n48 + n47 + n46) : 0;
assign level3 = (position[7:4] >= 2) ? (n35 + n34 + n33 + n32 + n31 + n30 + n29) : 0;
assign level4 = (position[7:4] >= 1) ? (n19 + n18 + n17 + n16 + n15 + n14 + n13) : 0;
assign level5 = n4  + n3 + n2 + n1 + center + p1 + p2 + p3 + p4;
assign level6 = (position[7:4] <= 15 - 1) ? (p19 + p18 + p17 + p16 + p15 + p14 + p13) : 0;
assign level7 = (position[7:4] <= 15 - 2) ? (p35 + p34 + p33 + p32 + p31 + p30 + p29) : 0;
assign level8 = (position[7:4] <= 15 - 3) ? (p50 + p49 + p48 + p47 + p46) : 0;
assign level9 = (position[7:4] <= 15 - 4) ? p64 : 0;

reg [7:0] anotherTarget;

wire [6:0] range [0:23];

assign range[0] = 64;
assign range[1] = 50;
assign range[2] = 49;
assign range[3] = 48;
assign range[4] = 47;
assign range[5] = 46;
assign range[6] = 35;
assign range[7] = 34;
assign range[8] = 33;
assign range[9] = 32;
assign range[10] = 31;
assign range[11] = 30;
assign range[12] = 29;
assign range[13] = 19;
assign range[14] = 18;
assign range[15] = 17;
assign range[16] = 16;
assign range[17] = 15;
assign range[18] = 14;
assign range[19] = 13;
assign range[20] = 4;
assign range[21] = 3;
assign range[22] = 2;
assign range[23] = 1;

wire i_n_50, 
assign center  = item[position - 0];
assign n64 = 					   item[position - 64];                            
assign n50 = 					   position[3:0] >= 2 && item[position - 50];      
assign n49 = 					   position[3:0] >= 1 && item[position - 49];      
assign n48 = 					   item[position - 48];                            
assign n47 = 					   position[3:0] <= 15 - 1 && item[position - 47]; 
assign n46 = 					   position[3:0] <= 15 - 2 && item[position - 46]; 
assign n35 = 					   position[3:0] >= 3 && item[position - 35];      
assign n34 = 					   position[3:0] >= 2 && item[position - 34];      
assign n33 = 					   position[3:0] >= 1 && item[position - 33];      
assign n32 = 					   item[position - 32];                            
assign n31 = 					   position[3:0] <= 15 - 1 && item[position - 31]; 
assign n30 = 					   position[3:0] <= 15 - 2 && item[position - 30]; 
assign n29 = 					   position[3:0] <= 15 - 3 && item[position - 29]; 
assign n19 = 					   position[3:0] >= 3 && item[position - 19];      
assign n18 = 					   position[3:0] >= 2 && item[position - 18];      
assign n17 = 					   position[3:0] >= 1 && item[position - 17];      
assign n16 = 					   item[position - 16];                            
assign n15 = 					   position[3:0] <= 15 - 1 && item[position - 15]; 
assign n14 = 					   position[3:0] <= 15 - 2 && item[position - 14]; 
assign n13 = 					   position[3:0] <= 15 - 3 && item[position - 13]; 
assign n4  = 					   position[3:0] >= 4 && item[position -  4];      
assign n3  =                       position[3:0] >= 3 && item[position -  3];      
assign n2  =                       position[3:0] >= 2 && item[position -  2];      
assign n1  =                       position[3:0] >= 1 && item[position -  1];      

assign p1  =                            position[3:0] <= 15 - 1 && item[position +  1]; 
assign p2  = 	                        position[3:0] <= 15 - 2 && item[position +  2]; 
assign p3  = 							position[3:0] <= 15 - 3 && item[position +  3]; 
assign p4  = 							position[3:0] <= 15 - 4 && item[position +  4]; 
assign p19 = 						    position[3:0] <= 15 - 3 && item[position + 19]; 
assign p18 = 						    position[3:0] <= 15 - 2 && item[position + 18]; 
assign p17 = 						    position[3:0] <= 15 - 1 && item[position + 17]; 
assign p16 = 						    item[position + 16];                            
assign p15 = 						    position[3:0] >= 1 && item[position + 15];      
assign p14 = 						    position[3:0] >= 2 && item[position + 14];      
assign p13 = 						    position[3:0] >= 3 && item[position + 13];      
assign p35 = 						    position[3:0] <= 15 - 3 && item[position + 35]; 
assign p34 = 						    position[3:0] <= 15 - 2 && item[position + 34]; 
assign p33 = 						    position[3:0] <= 15 - 1 && item[position + 33]; 
assign p32 = 						    item[position + 32];                            
assign p31 = 						    position[3:0] >= 1 && item[position + 31];      
assign p30 = 						    position[3:0] >= 2 && item[position + 30];      
assign p29 = 						    position[3:0] >= 3 && item[position + 29];      
assign p50 = 						    position[3:0] <= 15 - 2 && item[position + 50]; 
assign p49 = 						    position[3:0] <= 15 - 1 && item[position + 49]; 
assign p48 = 						    item[position + 48];                            
assign p47 = 						    position[3:0] >= 1 && item[position + 47];      
assign p46 = 						    position[3:0] >= 2 && item[position + 46];      
assign p64 = 						    item[position + 64];                            

//assign sum4x4 = level1 + level2 + level3 + level4 + level5 + level6 + level7 + level8 + level9;

//aim: to find two point to maximize cover point
//my aim: finish it in 150 lines
always@(state, finishGetdata, finishIteration,finishUNMasking, finishMasking, gotoUNMasking)begin
	case(state)
		GETDATA:
			if(finishGetdata == 1)
				nextState = ITERATION;
			else
				nextState = GETDATA;
		ITERATION:
			if(gotoUNMasking == 1)
				nextState = UNMASKING;
			else if(finishIteration == 1)
				nextState = STOP;
			else 
				nextState = ITERATION;
		UNMASKING:
			if(finishUNMasking)
				nextState = MASKING;
			else
				nextState = UNMASKING;
		MASKING:
			if(finishMasking)
				nextState = ITERATION;
			else
				nextState = MASKING;
		default:
			nextState = 1;
	endcase
end
always@(posedge CLK)begin
	if(RST)
		state = 1;
	else
		state = nextState;
end
integer i;
always@(posedge CLK)begin
	finishGetdata <= 0;
	finishIteration <= 0;
	finishMasking <= 0;
	finishUNMasking <= 0;
	gotoUNMasking <= 0;
	DONE <= 0;
	
	if(RST)begin
		totalsum <= 0;
		count <= 0;
		target1sum <= 0;
		target2sum <= 0;
		targetChoose <= 0;
		positionChange <= 0;
		anotherTarget = 0;
		DONE <= 0;
		position <= 0;
		allStop <= 0;
		maskcenter <= 0;
		firstUNMask <= 1;
		for(i = 0;i <= 255 ;i =i + 1)begin
			item[i] <= 0;
			
		end
		for(i = 0;i <= 23;i=i+ 1)begin
			maskn[i] <= 0;
			maskp[i] <= 0;
		end
	end
	else
		case(state)
			GETDATA:begin
				position <= position + 1;
				item[X + 16 * Y] <= 1;
				if(position == 40)begin
					position <= 0;
					finishGetdata <= 1;
				end
				else begin
					position <= position + 1;
					finishGetdata <= 0;
				end
					
			end
			ITERATION:begin
				//iteration: first step is to find the maximum sum as first point
				if(targetChoose == 0)begin
					if((sum4x4 + target2sum) > totalsum)begin
						totalsum <= sum4x4 + target2sum;
						C1X <= position[3:0];
						C1Y <= position[7:4];
						
						
						positionRecord <= position;
						positionChange <= 1;
							
						
					end
					else if((sum4x4 + target2sum) == totalsum)begin
						C1X <= position[3:0];
						C1Y <= position[7:4];
						if(position > positionRecord)begin
							positionRecord <= position;
							positionChange <= 1;
						end
						else begin
							positionRecord <= positionRecord;
							positionChange <= positionChange;
						end
						
					end
					else begin
						C1X <= C1X;
						C1Y <= C1Y;
					end
					
					if(position == 255)begin
						positionChange <= 0;
						position <= C1X + 16 * C1Y;
						
						if(positionChange == 0&& allStop)begin
							finishIteration <= 1;
							
						end
						else if(positionChange == 0)begin
							
							allStop <= 1;
							targetChoose <= 1;
							gotoUNMasking <= 1;
						end
						else begin
							allStop <= 0;
							targetChoose <= 1;
							gotoUNMasking <= 1;
						end
					end
					else
						position <= position + 1;
				end
				else begin
					if((sum4x4 + target1sum) > totalsum)begin
						C2X <= position[3:0];
						C2Y <= position[7:4];
						totalsum <= sum4x4 + target1sum;
						positionRecord <= position;
						positionChange <= 1;
						
					end
					else if((sum4x4 + target1sum) == totalsum)begin
						C2X <= position[3:0];
						C2Y <= position[7:4];
						if(position > positionRecord)begin
							positionRecord <= position;
							positionChange <= 1;
						end
						else begin
							positionRecord <= positionRecord;
							positionChange <= positionChange;
							
						end
					end
					else begin
						C2X <= C2X;
						C2Y <= C2Y;
					end
					
					if(position == 255)begin
						position <= C2X + 16 * C2Y;
						positionChange <= 0;
						if(positionChange == 0&& allStop)begin
							finishIteration <= 1;
						end
						else if(positionChange == 0)begin
							
							allStop <= 1;
							targetChoose <= 0;
							gotoUNMasking <= 1;
						end
						else begin
							allStop <= 0;
							
							targetChoose <= 0;
							gotoUNMasking <= 1;
						end
					end
					else
						position <= position + 1;
				end
			end
			MASKING:begin
				//make masking
				if(targetChoose == 1)
					anotherTarget = C1X + 16 * C1Y;
				else
					anotherTarget = C2X + 16 * C2Y;
				
				item[anotherTarget - range[count]] <= 0;
				maskn[count] <= item[anotherTarget - range[count]];
				item[anotherTarget + range[count]] <= 0;
				maskp[count] <= item[anotherTarget + range[count]];
				item[anotherTarget] <= 0;

				if(count == 0)begin
					maskcenter <= item[anotherTarget];
					if(targetChoose == 1)begin
						target1sum <= sum4x4;
						target2sum <= target2sum;
							
					end
					else if(targetChoose == 0)begin
						target2sum <= sum4x4;
						target1sum <= target1sum;	
					end
					else begin
						target1sum <= target1sum;
						target2sum <= target2sum;
							
					end
				end
				if(count < 23)
					count <= count + 1;
				else begin
					count <= 0;
						
					finishMasking <= 1;
					position <= 0;
				end
				
			end
			UNMASKING:begin
				//make unmasking
				if(firstUNMask)begin
					finishUNMasking <= 1;
					firstUNMask <= 0;
				end
				else begin
					item[anotherTarget - range[count]] <= maskn[count];
					item[anotherTarget + range[count]] <= maskp[count];
					item[anotherTarget] <= maskcenter;
				
					if(count < 23)
						count <= count + 1;
					else begin
						count <= 0;
						finishUNMasking <= 1;
						
					end
				end
			
			end
			STOP:begin
				DONE <= 1;
		
				totalsum <= 0;
				count <= 0;
				target1sum <= 0;
				target2sum <= 0;
				targetChoose <= 0;
				positionChange <= 0;
				anotherTarget = 0;
				position <= 0;
				allStop <= 0;
				maskcenter <= 0;
				positionRecord <= 0;
				firstUNMask <= 1;
				for(i = 0;i <= 255 ;i =i + 1)begin
					item[i] <= 0;
					
				end
				for(i = 0;i <= 23;i=i+ 1)begin
					maskn[i] <= 0;
					maskp[i] <= 0;
				end
			end
		endcase
end
endmodule


