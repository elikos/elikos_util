/*******************************************************************************
* Interface (classe abstraite pure)
*
* Implémentation super simple d'un observeur en c++. Devrait probablement être
* déplacé dans une libraire externe (FIXME).
*
* @author Arnaud Paré-Vogt
*******************************************************************************/
//on prie qu'il n'y ait pas d'autres implémentations...
#ifndef OBSERVER_H
#define OBSERVER_H

class Observer{

public:

    /***************************************************************************
    * Méthode qui devrait être appelée par l'observé si des changements qui 
    * méritent d'être analysés par l'observeur surviennent.
    ***************************************************************************/
    virtual void update() = 0;

};


#endif
