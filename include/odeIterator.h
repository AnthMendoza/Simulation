#ifndef ODEITERATOR_H
#define ODEITERATOR_H
    
    void Ode(float force , float mass , float timeStep ,float &velocity ,float &position);

    float rotationalOde(float moment , float MOI , float timeStep ,float &angularVelocity );


#endif