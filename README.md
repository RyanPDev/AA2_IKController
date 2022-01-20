TEAM DESCRIPTION
- Ryan Palazón --> ryan.palazon@enti.cat
- Marc Romera --> marc.romera.rodriguez@enti.cat

EXERCISE 1
--> script: MovingBall
--> location: Ball prefab

We calculate using projectile motion trajectory equation and we update the ball position through Euler.
As initially we don't know the initial velocity, we isolate it in the equation and apply the consequent operations this way:

iSpeed = 1 / Mathf.Cos(angle) * Mathf.Sqrt((0.5f * -acceleration.y * Mathf.Pow(distance, 2)) / (distance * Mathf.Tan(angle) + yOffset));

Vector3 velocity = new Vector3(0, iSpeed * Mathf.Sin(angle) * 2, iSpeed * Mathf.Cos(angle));

float angleBetweenObjects = Vector3.Angle(Vector3.forward, unitDirection) * (endPosition.x > transform.position.x ? 1 : -1);

iVel = Quaternion.AngleAxis(angleBetweenObjects, Vector3.up) * velocity;

EXERCISE 2
--> script: MovingBall and IKScorpion (to call it)
--> location: Ball prefab and Scorpion prefab

Did not have enough time to implement it as a whole.
A fairly rough idea of what we started can be seen in the MovingBall script, where we use the formula we found investigating: 
currentVelocity += 0.5f * effectStrength * Vector3.Cross(transform.right, currentVelocity);

where the magnus force will always be perpendicular to the ball's rotation and velocity and will constantly variate its velocity vector

EXERCISE 3
--> script: MyScorpionController
--> location: IKScorpion instanciates it

In this exercise, we changed the way the legs do their "lerp" to move from one point to another. We added a raycast to place the legs on top of the ground and then we also added two lerps.
One of them made the legs go up until the first half of the "total horizontal lerp" and the other one made the legs go down until the end horizontal lerp.
Formula used: 
a = Vector3.Lerp(auxPrevBases[i].position, auxFutureBases[i].position, t[i]); --> Horizontal lerp
a.y = Vector3.Lerp(auxPrevBases[i].position, auxMidPointBases[i], t[i] * 2).y; --> Lerp on Y axis in the first half of the horizontal movement, making the leg go up
a.y = Vector3.Lerp(auxMidPointBases[i], auxFutureBases[i].position, t[i] - (1 - t[i])).y; --> Lerp on Y axis in the last half of the horizontal movement, making the leg go down

Then we updated first the body to go up and down with its legs. We did it adding to the body position the average height offset of all the legs
We also added body rotation comparing the vectors from both legs infront and behind and then calculating the variation of the angle between those two vectors. Then we rotate the body in the direction of the axis of the rotation (axis forward and right)

EXERCISE 4
--> Animations done in Motion Builder
--> location: RobotKyle folder --> Model folder
We use an animator controller to do the transitions