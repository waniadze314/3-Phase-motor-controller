void establish_commutation();
float* clarke_park_transform(float* current, float angle);
float* inverse_clarke_park_transform(float* DQ, float angle);
void regulate_angle();
float* regulate_DQ();
float angle_to_rad(float angle);

void set_KP(uint16_t);
void set_KI(uint16_t);
void set_KD(uint16_t);
void set_Q_KP(uint16_t);
void set_Q_KI(uint16_t);
void set_D_KP(uint16_t);
void set_D_KI(uint16_t);