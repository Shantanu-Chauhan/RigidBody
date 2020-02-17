# RigidBody
3D physics engine that supports stable stacking of Cubes at 60 fps (150+ cubes with stack height of 5 and maximum stable stack height of 10), friction model and Joint Simulation using Sequential Impulse Based Constraint Solver (Ball & Socket and Hinge Joint)

	Project dealt with Multiple Rigid Body (Cubes) collision detection and resolution and held 60 fps with 100+ objects

	Broad phase consisted of Dynamic AABB tree with fattened up AABB to contain the actual body

	Narrow Phase used SAT (Separating Axis Test) to detect collision between 2 bodies

	Sutherland Hodgeman clipping was used to get all the contact points between 2 bodies in One Shot

	The solver was Projected Gauss-Seidel (PGS)
