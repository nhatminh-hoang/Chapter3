#include "pch.h"
#include"CRobot3DOF.h"

CRobot3DOF::CRobot3DOF()
{
	InitParams();
	InitTrajectory();
	SolvingPosition();
	SolvingVelocity();
	SolvingAcceleration();
}
void CRobot3DOF::InitParams()
{	
	m_a1 = 100;
	m_a2 = 300;
	m_a3 = 200;

	m_theta1 = 0;
	m_theta2 = 0;
	m_theta3 = 0;
}
void CRobot3DOF::InitTrajectory()
{
	int t = 100;
	for (int i = 0; i <= t; i++)
	{
		m_xE.Add(100 + 2*i);
		m_yE.Add(50 + 0.75*i);
		m_zE.Add(150 + i);

		m_vxE.Add(1);
		m_vyE.Add(1);
		m_vzE.Add(1);

		m_axE.Add(0);
		m_ayE.Add(0);
		m_azE.Add(0);
	}
	
}

//CRobot3DOF* pRobot3DOF = NULL;

void DefineFf(smatrix& F, vectorm& f, const vectorm& x)
{
	// input F_jacobian Matrix using Matlab
	// F is jacobian matrix
	// f is vector f

	F.SetSize(3);
	f.SetSize(3);

	F(0,0) = pRobot3DOF->m_a3 * sin(x[0]) * sin(x[1]) * sin(x[2])
		- pRobot3DOF->m_a3 * sin(x[0]) * cos(x[1]) * cos(x[2])
		- pRobot3DOF->m_a2 * sin(x[0]) * cos(x[1]);

	F(0,1) = -pRobot3DOF->m_a2 * cos(x[0]) * sin(x[1])
		- pRobot3DOF->m_a3 * cos(x[0]) * cos(x[1]) * sin(x[2])
		- pRobot3DOF->m_a3 * cos(x[0]) * sin(x[1]) * cos(x[2]);

	F(0,2) = -pRobot3DOF->m_a3 * cos(x[0]) * cos(x[1]) * sin(x[2])
		- pRobot3DOF->m_a3 * cos(x[0]) * sin(x[1]) * cos(x[2]);

	F(1,0) = pRobot3DOF->m_a2 * cos(x[0]) * cos(x[1])
		+ pRobot3DOF->m_a3 * cos(x[0]) * cos(x[1]) * cos(x[2])
		- pRobot3DOF->m_a3 * cos(x[0]) * sin(x[1]) * sin(x[2]);

	F(1,1) = -pRobot3DOF->m_a2 * sin(x[0]) * sin(x[1])
		- pRobot3DOF->m_a3 * sin(x[0]) * cos(x[1]) * sin(x[2])
		- pRobot3DOF->m_a3 * sin(x[0]) * sin(x[1]) * cos(x[2]);

	F(1,2) = -pRobot3DOF->m_a3 * sin(x[0]) * cos(x[1]) * sin(x[2])
		- pRobot3DOF->m_a3 * sin(x[0]) * sin(x[1]) * cos(x[2]);

	F(2,0) = 0;

	F(2,1) = pRobot3DOF->m_a2 * cos(x[1])
		+ pRobot3DOF->m_a3 * cos(x[1]) * cos(x[2])
		- pRobot3DOF->m_a3 * sin(x[1]) * sin(x[2]);

	F(2,2) = pRobot3DOF->m_a3 * cos(x[1]) * cos(x[2])
		- pRobot3DOF->m_a3 * sin(x[1]) * sin(x[2]);

	f(0) = pRobot3DOF->m_a2 * cos(x[0]) * cos(x[1])
		+ pRobot3DOF->m_a3 * cos(x[0]) * cos(x[1]) * cos(x[2])
		- pRobot3DOF->m_a3 * cos(x[0]) * sin(x[1]) * sin(x[2])
		- pRobot3DOF->m_xE[pRobot3DOF->m_currentIndex];

	f(1) = pRobot3DOF->m_a2 * sin(x[0]) * cos(x[1])
		+ pRobot3DOF->m_a3 * sin(x[0]) * cos(x[1]) * cos(x[2])
		- pRobot3DOF->m_a3 * sin(x[0]) * sin(x[1]) * sin(x[2])
		- pRobot3DOF->m_yE[pRobot3DOF->m_currentIndex];

	f(2) = pRobot3DOF->m_a1 + pRobot3DOF->m_a2 * sin(x[1])
		+ pRobot3DOF->m_a3 * cos(x[1]) * sin(x[2])
		+ pRobot3DOF->m_a3 * sin(x[1]) * cos(x[2])
		- pRobot3DOF->m_zE[pRobot3DOF->m_currentIndex];
}

bool CRobot3DOF::SolvingPosition()
{
	// TODO: Add your implementation code here.
	vectorm x(3);
	x[0] = PI / 4.0;
	x[1] = PI / 5.0;
	x[2] = PI / 6.0;

	pRobot3DOF = this;
	m_qtheta1.RemoveAll();
	m_qtheta2.RemoveAll();
	m_qtheta3.RemoveAll();

	bool res = true;

	for (int i = 0; i < m_xE.GetSize(); i++)
	{
		m_currentIndex = i;
		if (Newton_Raphson(x, DefineFf) == 0)
		{
			m_qtheta1.Add(x[0]);
			m_qtheta2.Add(x[1]);
			m_qtheta3.Add(x[2]);
		}
		else { return res = false; }
	}

	return res;
}


bool CRobot3DOF::SolvingVelocity()
{
	// TODO: Add your implementation code here.
	bool res = true;
	m_vtheta1.RemoveAll();
	m_vtheta2.RemoveAll();
	m_vtheta3.RemoveAll();

	smatrix A(3);
	vectorm x(3);
	vectorm b(3); // b = A * x

	for (int i = 0; i < m_qtheta1.GetSize(); i++)
	{
		// x = [theta1, theta2, theta3]
		// b = [vtheta1, vtheta2, vtheta3]
		// A = [dF/dtheta1, dF/dtheta2, dF/dtheta3]
		//...

		if (Gauss_Jordan(A, x, b) != 0) res = false;
		m_vtheta1.Add(x[0]);
		m_vtheta2.Add(x[1]);
		m_vtheta3.Add(x[2]);

	}

	return res;
}


bool CRobot3DOF::SolvingAcceleration()
{
	// TODO: Add your implementation code here.
	return false;
}
