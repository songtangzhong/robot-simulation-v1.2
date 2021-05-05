#ifndef ROBOT_BASIC_MACRO_H_
#define ROBOT_BASIC_MACRO_H_

#define ARM_DOF 7
#define ARM_SHM_KEY 1234
#define ARM_SEM_KEY 1235

#define END_EFF_DOF 2
#define END_EFF_SHM_KEY 2234
#define END_EFF_SEM_KEY 2235

#define ROBOT_STATE_SHM_KEY 3234
#define ROBOT_STATE_SEM_KEY 3235

// Define if the end-effector has been used or not in other packages,
// but has no influence on this package.
#define END_EFF_TRUE 1 // 1(use) 0(not use)

#endif