void establish_commutation();
float* clarke_park_transform(float* current, float angle);
float* inverse_clarke_park_transform(float* DQ, float angle);
void regulate_angle();
float* regulate_DQ();
float angle_to_rad(float angle);