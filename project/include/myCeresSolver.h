#ifndef MYCERES_H_
#define MYCERES_H_


#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <vector>



class myCeresSolver
{
public:
	myCeresSolver();
	~myCeresSolver();
	void set_solver_params();
	void set_estimate(double* rot, double  s, double* t);
	void get_estimate(double* rot, double* s, double* t);
	bool add_residuals(double* tg, double* tl);
	void check_math(double* tg, double* tl);
	void solve();

	void hold_scale();
	void hold_rotation();
	void hold_trnslation();


private:


	struct F1 {

		F1(double Xg_, double Yg_, double Zg_,double Xl_, double Yl_, double Zl_):
			Xg(Xg_), Yg(Yg_),Zg(Zg_),
			Xl(Xl_), Yl(Yl_), Zl(Zl_){}

	  template <typename T> bool operator()(const T* const sRT,
						  	  T* residual) const {
	    // f1
		    T ip[3];
		    ip[0] = T(Xg);
		    ip[1] = T(Yg);
		    ip[2] = T(Zg);

		    T op[3];

		    //rotate
		    ceres::AngleAxisRotatePoint(sRT, ip, op);

		    //scale
		    ip[0] *= sRT[6];
		    ip[1] *= sRT[6];
		    ip[2] *= sRT[6];

		    //translate
		    op[0] += sRT[3];
		    op[1] += sRT[4];
		    op[2] += sRT[5];


		  residual[0] = (T)Xl -  op[0];
		  residual[1] = (T)Yl -  op[1];
		  residual[2] = (T)Zl -  op[2];


	    return true;
	  }

	private:
	    const double Xg, Yg, Zg;
	    const double Xl, Yl, Zl;
	};


	ceres::Problem problem;
	ceres::Solver::Options options;
	ceres::Solver::Summary summary;

	double sRT_[7];


};

#endif

