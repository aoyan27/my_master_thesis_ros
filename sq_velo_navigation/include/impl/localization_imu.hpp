Localization::Localization(){
//	sw.start();
	x= y= z= qw= qx= qy= qz= pitch = yaw = v = w = 0.0;
	time = 0.0;
}

Localization::~Localization(){
}

void Localization::showState(){
	cout << "x: " << x << ", y: " << y << ", yaw: " << yaw << endl;
}

void Localization::start(){
	sw.start();
}

void Localization::altering(){
	double dt = sw.getTime();
	sw.reset();
	yaw += dt * w; 
	//if(yaw<-M_PI)yaw += 2*M_PI;
	//if(yaw>M_PI)yaw -= 2*M_PI;
	x += dt * v * cos(pitch) * cos(yaw);
	y += dt * v * cos(pitch) * sin(yaw);
	z += dt * v * sin(pitch);
}
void Localization::altering2(){
	double dt = sw.getTime();
	sw.reset();
	yaw += dt * w; 
	//if(yaw<-M_PI)yaw += 2*M_PI;
	//if(yaw>M_PI)yaw -= 2*M_PI;
	x += dt * v * cos(yaw);
	y += dt * v * sin(yaw);
}
void Localization::altering3(){
	double dt = sw.getTime();
	sw.reset();
	//if(yaw<-M_PI)yaw += 2*M_PI;
	//if(yaw>M_PI)yaw -= 2*M_PI;
	x += dt * v * cos(pitch) * cos(yaw);
	y += dt * v * cos(pitch) * sin(yaw);
	z += dt * v * sin(pitch);
}
