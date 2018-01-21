Localization::Localization(){
//	sw.start();
	x = y = pitch = yaw = v = w = 0.0;
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

void Localization::gettime(){
	time = sw.getTime();
	sw.reset();
}

void Localization::altering(){
	// double dt = 3.0 * sw.getTime();
	double dt = sw.getTime();
	sw.reset();
	yaw += dt * w; 
	// if(yaw<-M_PI)yaw += 2*M_PI;
	//if(yaw>M_PI)yaw -= 2*M_PI;
	x += dt * v * cos(pitch) * cos(yaw);
	y += dt * v * cos(pitch) * sin(yaw);
}
void Localization::altering2(){
	// double dt = 3.0 * sw.getTime();
	double dt = sw.getTime();
	sw.reset();
	// yaw += dt * w; 
	//if(yaw<-M_PI)yaw += 2*M_PI;
	//if(yaw>M_PI)yaw -= 2*M_PI;
	// x += dt * v * cos(yaw);
	// y += dt * v * sin(yaw);
	x += dt * v * cos(pitch) * cos(yaw);
	y += dt * v * cos(pitch) * sin(yaw);
}
