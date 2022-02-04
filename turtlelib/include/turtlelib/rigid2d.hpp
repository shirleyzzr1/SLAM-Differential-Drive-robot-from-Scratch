#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects
#include <cmath>
namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief turn any angle into the equivalent angle in the interval(-pi,pi)
    /// \param rad - angle in radians
    /// \return double rad - the angle in interval(-pi,pi)
    double normalize_angle(double rad);

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if (abs(d1-d2)<epsilon)return true;
        return false;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return deg/180*PI;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad/PI*180;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief add vector to the original one
        /// rhs- input vector
        Vector2D & operator +=(const Vector2D & rhs);

        /// \brief minus an vector to the original one
        /// rhs - input vector
        Vector2D & operator -=(const Vector2D & rhs);

        /// \brief multiply the vector with a scalar
        /// rhs - input vector
        Vector2D & operator *=(const double scalar);
    };

    /// \brief normalize the 2D vector
    /// v input vector
    /// output the normalized vector
    Vector2D Vector2Dnormalize(Vector2D v);

    /// \brief add two vectors
    /// lhs, rhs - input vectors
    Vector2D & operator+(Vector2D &lhs,const Vector2D & rhs);

    /// \brief vectors subtraction
    /// lhs, rhs- input vectors
    Vector2D & operator-(Vector2D &lhs,const Vector2D & rhs);

    /// \brief multiplay vector by a scalar on the right
    /// lhs, an vector
    /// scalar- an scalar
    Vector2D & operator*(Vector2D &lhs,const double scalar);

    /// \brief multiplay vector by a scalar on the left
    /// lhs, an vector
    /// scalar- an scalar
    Vector2D & operator*(const double scalar,Vector2D &lhs);

    /// \brief the dot product of two vectors
    /// -lhs, rhs two input vectors
    /// \return the dot product
    double dot(const Vector2D &lhs ,const Vector2D & rhs);

    /// \brief compute the magnitude of the vector
    /// vector- the input vector
    /// \return the magnitude of the vector
    double magnitude(const Vector2D& vector);

    /// \brief compute the angle between two vectors
    /// input rwo vectors
    /// return in radians
    double angle(const Vector2D& , const Vector2D&);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// We have lower level control however. For example:
    /// peek looks at the next unprocessed character in the buffer without removing it
    /// get removes the next unprocessed character from the buffer.
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief A 2-Dimensional twist
    struct Twist2D{
        /// \brief the angular component of a twist
        double thetadot;
        /// \brief the linear x component of a twist
        double xdot;
        /// \brief the linear y component of t twist
        double ydot;
    };

    
    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    private:
        double radians;
        Vector2D trans;
        
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply a convert to a twist2D from one frame to another
        /// \param tw - the twist to convert
        /// \param a new twist in the new coordinate system
        Twist2D operator()(Twist2D tw)const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
    };
    /// \brief output a 3 dimensional twist as [thetadot xdot ydot]
    /// \param os - an output stream
    /// \param tf - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw);

    /// \brief input a 3 dimensional twist
    /// \param is - an input stream
    /// \param tw - the twist to input
    std::istream & operator>>(std::istream & is, Twist2D & tw);   

    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    ///\brief compute the transformation corresponding to a rigid body follow
    /// twist at a given time-unit
    /// input - 2d twist
    Transform2D integrate_twist(const Twist2D&);

}

#endif
