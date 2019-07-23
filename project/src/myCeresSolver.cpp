#include "myCeresSolver.h"


myCeresSolver::myCeresSolver()
{
}
myCeresSolver::~myCeresSolver()
{

}

void myCeresSolver::set_solver_params()
{
	  options.max_num_iterations = 100;
	  options.linear_solver_type = ceres::DENSE_SCHUR;
	  options.minimizer_progress_to_stdout = true;
}


void myCeresSolver::set_estimate(double* r, double s, double* t)
{

	//change r to column major
	double r_[9];
	r_[0] = r[0];  	r_[1] = r[3];	r_[2] = r[6];
	r_[3] = r[1];	r_[4] = r[4];	r_[5] = r[7];   
	r_[6] = r[2];	r_[7] = r[5];	r_[8] = r[8];

	double R[3];

	ceres::RotationMatrixToAngleAxis(r_,R);
	
	//rotation in angle-axis
	sRT_[0] = R[0];
	sRT_[1] = R[1];
	sRT_[2] = R[2];

	//translation
	sRT_[3] = t[0];
	sRT_[4] = t[1];
	sRT_[5] = t[2];

	//scale
	sRT_[6]  = s;

}

//adds residual between from and to points
bool myCeresSolver::add_residuals(double* Pf,double* Pt)
{

	//check_math(Pf,Pt);
	//ceres::ScaledLoss* loss_function = new ceres::ScaledLoss(new ceres::HuberLoss(1.0), 1, ceres::TAKE_OWNERSHIP);
	ceres::CostFunction* cost_function =new ceres::AutoDiffCostFunction<F1, 3, 7>(new F1(Pf[0],Pf[1],Pf[2],Pt[0],Pt[1],Pt[2]));
	problem.AddResidualBlock(cost_function, NULL, sRT_);
	return true;
}


void myCeresSolver::check_math(double* tg, double* tl)
{

	double ip[3];
	double op[3];

	ip[0] = tg[0];
	ip[1] = tg[1];
	ip[2] = tg[2];

	ceres::AngleAxisRotatePoint(sRT_, ip, op);

	//scale
	op[0] *= sRT_[6];
	op[1] *= sRT_[6];
	op[2] *= sRT_[6];
	//translate
	op[0] += sRT_[3];
	op[1] += sRT_[4];
	op[2] += sRT_[5];

	std::cout << "calc " << op[0] << " " << op[1] <<  " "  << op[2] << std::endl;
	std::cout << "obsv " << tl[0] << " " << tl[1] <<  " "  << tl[2] << std::endl;
	std::cout << std::endl;

}



void myCeresSolver::get_estimate(double* eR, double* eS, double* eT )
{

	double R[3];

	//estimated rotation matrix
	R[0] = sRT_[0];
	R[1] = sRT_[1];
	R[2] = sRT_[2];
	double r[9];
	
	//estimated rotation
	ceres::AngleAxisToRotationMatrix(R,r);
	eR[0] = r[0];  	eR[1] = r[3];	eR[2] = r[6];
	eR[3] = r[1];	eR[4] = r[4];	eR[5] = r[7];   
	eR[6] = r[2];	eR[7] = r[5];	eR[8] = r[8];

	//estimated translation
	eT[0] = sRT_[3];
	eT[1] = sRT_[4];
	eT[2] = sRT_[5];

	//estimated scale
	eS[0] = sRT_[6];

}


void myCeresSolver::hold_scale()
{
	problem.SetParameterization(sRT_, new ceres::SubsetParameterization(7, {6}));
}

void myCeresSolver::hold_rotation()
{
	problem.SetParameterization(sRT_, new ceres::SubsetParameterization(7, {0,1,2}));
}

void myCeresSolver::hold_trnslation()
{
	problem.SetParameterization(sRT_, new ceres::SubsetParameterization(7, {3,4,5}));
}

void myCeresSolver::solve()
{
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";
}



