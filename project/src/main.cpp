#include "myCeresSolver.h"
#include <fstream>

typedef struct pt_type_
{
	double x;
	double y;
	double z;
}pt_type;


typedef struct trn_type_
{
	double r[9];
	double t[3];
	double s;
}trn_type;


void read_pts_file(std::string filename, std::vector<pt_type>& pts)
{
	std::fstream myfile(filename, std::ios_base::in);
	std::string line;

   	while (std::getline(myfile, line))
    	{	
        	pt_type pt;
        	std::stringstream ss(line);

		ss >> pt.x;
		ss >> pt.y;
		ss >> pt.z;

		pts.push_back(pt);
    	}
}

void read_ini_estimate(std::string filename, trn_type& est_set)
{
	std::fstream myfile(filename, std::ios_base::in);
	std::string line;

   	while (std::getline(myfile, line))
    	{	
        	pt_type pt;
        	std::stringstream ss(line);

		for(uint i=0;i<9;i++)
			ss >> est_set.r[i];

		for(uint i=0;i<3;i++)
			ss >> est_set.t[i];

		ss >> est_set.s;

    	}
}

void write_con_estimate(std::string filename, trn_type& est_get)
{
	 std::fstream myfile(filename, std::ios_base::out);

	//rotation
	for(uint i=0;i<9;i++)
		myfile << est_get.r[i] << " ";
	
	//translation
	for(uint i=0;i<3;i++)
		myfile << est_get.t[i] << " ";

	//scale
	myfile << est_get.s;

}

int main()
{
	//1. read the from points
	std::vector<pt_type> frompts;
	read_pts_file("../data/from_pts.txt",frompts);

	//2. read the to points
	std::vector<pt_type> topts;
	read_pts_file("../data/to_pts.txt",topts);
	
	//3. read the approximate transformation
	trn_type est_set;
	trn_type est_get;
	read_ini_estimate("../data/sim3_approximate.txt",est_set);

	//4. setup a ceres problem
	myCeresSolver* mySim3Solver = new myCeresSolver();
	mySim3Solver->set_solver_params();

	//5. set estimate 
	mySim3Solver->set_estimate(est_set.r,est_set.s,est_set.t);

	//6. feed the data
	for(uint i=0;i<frompts.size();i++)
	{
		double fpt[3] = {frompts[i].x,frompts[i].y,frompts[i].z};
		double tpt[3] = {topts[i].x,topts[i].y,topts[i].z};
	 	mySim3Solver->add_residuals(fpt,tpt);
	}
	//7. solve for s R and T
	mySim3Solver->solve();

	//8. get estimate
	mySim3Solver->get_estimate(est_get.r,&est_get.s,est_get.t);

	//9. write the estimated transformation	
	write_con_estimate("../data/sim3_converged.txt",est_get);

	return 0;
}

