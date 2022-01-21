# README

### Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

### Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

### Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   - Which of the methods would you implement and why?
   
   first method is to calculate directly, and give the vector the normalized value. Second one is to use a function to calculate, the input is the vector itself, the copy is passed and the output is a new normalized vector, without manipulating the old vector. The third one is to pass the vector by reference and modify on the input  itself. I think the first method is not a good one because sometimes you just do the calculation more than once and looks very tedious in the code. The third one saves the memory, but sometimes if you just want to keep origin vector and don't make any change, you can choose the second method.


2. What is the difference between a class and a struct in C++?

   For struct, the members and base classes are set to public by default, and for class, they are private by default. That's the main difference for them. For useful purpose, try to use struct if the data is related or they can vary indepently.

   For class, they normally has constructor and destructor.


3. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specific C++ core guidelines in your answer)?

   In C.1, it says that we should organize related data into structures, for x and y component in Vector2D, they are totally related. In C.2, it says that if we have invariant use class, and in Transform2DClass, all the member function can only be used when we set the value of the member radians and trans.


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

   We use explicit when declaring single-argument constructors to avoid unintended conversions.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   
   By defualt we should make member functions const in case some const class object wants to use this function. And also, in inv() function , we don't change the objects state., we do the inverse transformation and return the inversed one, that's why we can use const for this one.  However, for operator*(), the result is stored in this object.

# Sample Run of frame_main
```
shirleyzzr@superX:~/ws/src/slam-project-shirleyzzr1/turtlelib$ ./frame_main
Enter transform T_{a,b}:
90 0 1
Enter transform T_{b,c}:
90 1 0
T_{a,b}: deg: 90 x: 0 y: 1
T_{b,a}: deg: -90 x: -1 y: -6.12323e-17
T_{b,c}: deg: 90 x: 1 y: 0
T_{c,b}: deg: -90 x: -6.12323e-17 y: 1
T_{a,c}: deg: 180 x: 6.12323e-17 y: 2
T_{c,a}: deg: -180 x: -1.83697e-16 y: 2
Enter vector v_b:
1 1
v_bhat: [0.707107 0.707107]
v_a: [-1 2]
v_b: [1 1]
v_c: [1 1.11022e-16]
Enter twist V_b: 
1 1 1
V_a: [ 1 0 1 ]
V_b: [ 1 1 1 ]
V_c: [ 1 2 -1 ]
```
