dtmc
//version:V3 18/08/2021 by XZ.

//Listing all transition probabilities as variables.
const double p_G_B1 = 0.012569130216189;
const double p_G_B2 = 0.00215470803706098;
const double p_G_B3 = 0;
const double p_G_C = 0;
formula p_G_G = 1-p_G_B1-p_G_B2-p_G_B3-p_G_C;//the sum of all outgoing tran. prob. is 1

const double p_B1_G = 0.00982062330894879;
const double p_B1_B2 = 0.0283595550656378;
const double p_B1_B3 = 0.000100210441928049;
const double p_B1_C = 0;
formula p_B1_B1 = 1-p_B1_G-p_B1_B2-p_B1_B3-p_B1_C;

const double p_B2_G = 0.00768882858435097;
const double p_B2_B1 = 0.00705563093622795;
const double p_B2_B3 = 0.0088647670737223;
const double p_B2_C = 0;
formula p_B2_B2 = 1-p_B2_G-p_B2_B1-p_B2_B3-p_B2_C;

const double p_B3_G = 0.000772200772200772;
const double p_B3_B1 = 0;
const double p_B3_B2 = 0.0253539253539254;
const double p_B3_C = 0;
formula p_B3_B3 = 1-p_B3_G-p_B3_B1-p_B3_B2-p_B3_C;

module FailureProcByDistance
//This is the "failure process DTMC" abstracted based on the distance to obstacles
	s: [0..4] init 0; 	
	// 0 - G, ideal distance to the obstacle, "golden route".
	// 1 - B1, a bit close to the obstacle, benign failue level 1.
	// 2 - B2, somewhat close to the obstacle, benign failure level 2.
	// 3 - B3, too close to the obstacle, benign failure level 3.
	// 4 - C, crash into the obstacle, catastrophic failure.
	[tran] s=0 -> p_G_G:(s'=0)+p_G_B1:(s'=1)+p_G_B2:(s'=2)+p_G_B3:(s'=3)+p_G_C:(s'=4);
	[tran] s=1 -> p_B1_G:(s'=0)+p_B1_B1:(s'=1)+p_B1_B2:(s'=2)+p_B1_B3:(s'=3)+p_B1_C:(s'=4);
	[tran] s=2 -> p_B2_G:(s'=0)+p_B2_B1:(s'=1)+p_B2_B2:(s'=2)+p_B2_B3:(s'=3)+p_B2_C:(s'=4);
	[tran] s=3 -> p_B3_G:(s'=0)+p_B3_B1:(s'=1)+p_B3_B2:(s'=2)+p_B3_B3:(s'=3)+p_B3_C:(s'=4);
	[tran] s=4 -> 1:(s'=4);
endmodule

const int avg_mission_length=560;//average mission length, e.g., 500 steps.

module MissionStage
// This is the "mission-stage DTMC" represents the main stages of the mission.
	k:[0..1] init 0;//0: mission not finished; 1: mission finished
	[tran] k=0 -> 1/avg_mission_length:(k'=1)+(1-1/avg_mission_length):(k'=0);
endmodule

rewards "step"
	s<5 & k=0 : 1; //reward of 1 to every state of the model, thus representing steps..
endrewards

rewards "deviation"
	s=0 : 0; // 0m deviation from the 'golden route' (non-risky distance to obstacles)
	s=1 : 1; // 1m deviation from the 'golden route'
	s=2 : 2; // 2m deviation from the 'golden route'
	s=3 : 3; // 3m deviation from the 'golden route'
	s=4 : 0; // No practical meaning when crash, thus we use 0 as a placeholder.
endrewards

