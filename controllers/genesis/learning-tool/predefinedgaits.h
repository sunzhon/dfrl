#ifndef PREDEFINEDGAITS_H_
#define PREDEFINEDGAITS_H_
#include <vector>
using namespace std;
typedef std::vector<std::vector<double>> gaitMatrix;

enum GAIT{
	WALK_D = 0,
	WALK_L = 1,
	TROT = 2,
	PACE = 3,
	GALLOP = 4,
	NO_COUPLED =5
};
class PredefinedGait{
private:
	gaitMatrix walk_d_gait;
	gaitMatrix walk_l_gait;
	gaitMatrix trot_gait;
	gaitMatrix pace_gait;
	gaitMatrix gallop_gait;
	gaitMatrix decoupled_gait;
public:
	PredefinedGait(unsigned int legnum){
		walk_d_gait.resize(legnum);
		walk_l_gait.resize(legnum);
		trot_gait.resize(legnum);
		pace_gait.resize(legnum);
		gallop_gait.resize(legnum);
		decoupled_gait.resize(legnum);
		for(unsigned int i=0;i<legnum;i++){
			walk_d_gait.at(i).resize(legnum);
			walk_l_gait.at(i).resize(legnum);
			trot_gait.at(i).resize(legnum);
			pace_gait.at(i).resize(legnum);
			gallop_gait.at(i).resize(legnum);
			decoupled_gait.at(i).resize(legnum);
		}
		// gallop gait
	gallop_gait.at(0).at(0) = 0.0;
	gallop_gait.at(0).at(1) = -M_PI;
	gallop_gait.at(0).at(2) = 0.0;
	gallop_gait.at(0).at(3) = -M_PI;

	gallop_gait.at(1).at(0) = M_PI;
	gallop_gait.at(1).at(1) = 0.0;
	gallop_gait.at(1).at(2) = M_PI;
	gallop_gait.at(1).at(3) = 0.0;

	gallop_gait.at(2).at(0) = 0.0;
	gallop_gait.at(2).at(1) = -M_PI;
	gallop_gait.at(2).at(2) = 0.0;
	gallop_gait.at(2).at(3) = -M_PI;

	gallop_gait.at(3).at(0) = M_PI;
	gallop_gait.at(3).at(1) = 0.0;
	gallop_gait.at(3).at(2) = M_PI;
	gallop_gait.at(3).at(3) = 0.0;

	// pace gait
	pace_gait.at(0).at(0) = 0.0;
	pace_gait.at(0).at(1) = 0.0;
	pace_gait.at(0).at(2) = -M_PI;
	pace_gait.at(0).at(3) = -M_PI;

	pace_gait.at(1).at(0) = 0.0;
	pace_gait.at(1).at(1) = 0.0;
	pace_gait.at(1).at(2) = -M_PI;
	pace_gait.at(1).at(3) = -M_PI;

	pace_gait.at(2).at(0) = M_PI;
	pace_gait.at(2).at(1) = M_PI;
	pace_gait.at(2).at(2) = 0.0;
	pace_gait.at(2).at(3) = 0.0;

	pace_gait.at(3).at(0) = M_PI;
	pace_gait.at(3).at(1) = M_PI;
	pace_gait.at(3).at(2) = 0.0;
	pace_gait.at(3).at(3) = 0.0;

	// a walk_d gait
	walk_d_gait.at(0).at(0) = 0.0;
	walk_d_gait.at(0).at(1) = -1/2*M_PI;
	walk_d_gait.at(0).at(2) = -M_PI;
	walk_d_gait.at(0).at(3) = -3/2*M_PI;

	walk_d_gait.at(1).at(0) = 1/2*M_PI;
	walk_d_gait.at(1).at(1) = 0.0;
	walk_d_gait.at(1).at(2) = -1/2*M_PI;
	walk_d_gait.at(1).at(3) = -M_PI;

	walk_d_gait.at(2).at(0) = M_PI;
	walk_d_gait.at(2).at(1) = 1/2*M_PI;
	walk_d_gait.at(2).at(2) = 0.0;
	walk_d_gait.at(2).at(3) = -1/2*M_PI;

	walk_d_gait.at(3).at(0) = 3/2*M_PI;
	walk_d_gait.at(3).at(1) = M_PI;
	walk_d_gait.at(3).at(2) = 1/2*M_PI;
	walk_d_gait.at(3).at(3) = 0.0;

	// a walk_l gait
	walk_l_gait.at(0).at(0) = 0.0;
	walk_l_gait.at(0).at(1) = -3/2*M_PI;
	walk_l_gait.at(0).at(2) = -M_PI;
	walk_l_gait.at(0).at(3) = -1/2*M_PI;

	walk_l_gait.at(1).at(0) = 3/2*M_PI;
	walk_l_gait.at(1).at(1) = 0.0;
	walk_l_gait.at(1).at(2) = 1/2*M_PI;
	walk_l_gait.at(1).at(3) = M_PI;

	walk_l_gait.at(2).at(0) = M_PI;
	walk_l_gait.at(2).at(1) = -1/2*M_PI;
	walk_l_gait.at(2).at(2) = 0.0;
	walk_l_gait.at(2).at(3) = 1/2*M_PI;

	walk_l_gait.at(3).at(0) = 1/2*M_PI;
	walk_l_gait.at(3).at(1) = -M_PI;
	walk_l_gait.at(3).at(2) = -1/2*M_PI;
	walk_l_gait.at(3).at(3) = 0.0;

	// trot gait
	trot_gait.at(0).at(0) = 0.0;
	trot_gait.at(0).at(1) = -M_PI;
	trot_gait.at(0).at(2) = -M_PI;
	trot_gait.at(0).at(3) = 0.0;

	trot_gait.at(1).at(0) = M_PI;
	trot_gait.at(1).at(1) = 0.0;
	trot_gait.at(1).at(2) = 0.0;
	trot_gait.at(1).at(3) = M_PI;

	trot_gait.at(2).at(0) = M_PI;
	trot_gait.at(2).at(1) = 0.0;
	trot_gait.at(2).at(2) = 0.0;
	trot_gait.at(2).at(3) = M_PI;

	trot_gait.at(3).at(0) = 0.0;
	trot_gait.at(3).at(1) = -M_PI;
	trot_gait.at(3).at(2) = -M_PI;
	trot_gait.at(3).at(3) = 0.0;

	}

	gaitMatrix	getGait(GAIT gait){
		switch(gait){
			case WALK_D:
				return walk_d_gait;
			case WALK_L:
				return walk_l_gait;
			case TROT:
				return trot_gait;
			case GALLOP:
				return gallop_gait;
			case PACE:
				return pace_gait;
			case NO_COUPLED:
				return decoupled_gait;
			default:
				perror("error");
				exit(-1);
	
		}
	}
};






#endif
