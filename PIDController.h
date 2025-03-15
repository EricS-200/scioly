class PIDController{
  private: 
    float kP;
    float kI;
    float kD; 
    float kFF; 
    float deadband; 

    float integral;
    float lastError;

  public: 
    PIDController(float kp, float ki, float kd, float kff, float db);
    float calculate(float error);
    void reset();
};