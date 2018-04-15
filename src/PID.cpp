#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  this->p_error = cte;
  this->i_error += cte;
  this->d_error = cte - this->prev_cte;
  this->prev_cte = cte;
}

double PID::TotalError() {
	this->totalError += this->prev_cte * this->prev_cte; 
	return this->totalError; 
}

