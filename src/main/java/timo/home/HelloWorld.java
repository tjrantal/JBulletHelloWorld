/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 * HelloWorld port by: Clark Dorman
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package timo.home;

//import com.bulletphysics.collision.broadphase.AxisSweep3;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import javax.vecmath.Vector3f;

//Add air drag
import com.bulletphysics.dynamics.ActionInterface;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.linearmath.IDebugDraw;

/**
 * This is a Hello World program for running a basic Bullet physics simulation.
 * it is a direct translation of the C++ HelloWorld app.
 *
 * @author cdorman
 
 Modified by Timo Rantalainen 2019 tjrantal at gmail dot com
 Wanted to explore how to get access to implementing air drag and Magnus force with JBullet. ActionInterface seems to provide that opportunity
 
 */
public class HelloWorld
{

	//Could not figure out how to implement this...
	public static class AirDynamics extends ActionInterface{
		RigidBody obj;	//Pointer to the object to modify
		//Constructor
		public AirDynamics(RigidBody obj){
			this.obj = obj;
		}
		@Override
		public void updateAction(CollisionWorld collisionWorld, float deltaTimeStep){
			//Do air drag calculations here, just a demo that a side-ways force does take effect without removing collisions and gravity
			obj.applyCentralForce(new Vector3f(-10f,0f,0f));	//Apply magnus force
		}
		@Override
		public void debugDraw(IDebugDraw debugDrawer){
			//Do nothing
		}
	}

	public static void main(String[] args) {
		
		
		
		
		
		
		//collisionConfig = new btDefaultCollisionConfiguration();
		CollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();

		// use the default collision dispatcher. For parallel processing you
		// can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		//dispatcher = new btCollisionDispatcher(collisionConfig);
		CollisionDispatcher dispatcher = new CollisionDispatcher(
				collisionConfiguration);

		//broadphase = new btDbvtBroadphase();
		BroadphaseInterface broadphaseInterface = new DbvtBroadphase();
		
		// the default constraint solver. For parallel processing you can use a
		// different solver (see Extras/BulletMultiThreaded)
		//constraintSolver = new btSequentialImpulseConstraintSolver();
		SequentialImpulseConstraintSolver solver = new SequentialImpulseConstraintSolver();

		//dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, constraintSolver, collisionConfig);
		
		DiscreteDynamicsWorld dynamicsWorld = new DiscreteDynamicsWorld(
				dispatcher, broadphaseInterface, solver,
				collisionConfiguration);

		dynamicsWorld.setGravity(new Vector3f(0f, -9.81f, 0f));
		
		//Could add contactListener here?
		///contactListener = new MyContactListener();

		// create a few basic rigid bodies
		/*
			{0.218f},			//Ball
							{68f/2f,0.01f/2f,105f/2f},	//Pitch
		*/
		
		CollisionShape ballShape = new SphereShape(0.218f);	//Football pitch

		CollisionShape groundShape = new BoxShape(new Vector3f(68f/2f,0.01f/2f,105f/2f));	//Football pitch

		// keep track of the shapes, we release memory at exit.
		// make sure to re-use collision shapes among rigid bodies whenever
		// possible!
		ObjectArrayList<CollisionShape> collisionShapes = new ObjectArrayList<CollisionShape>();
		float[] objectMasses = new float[]{0f,0.436f};
		float[][] modelTransforms = new float[][]{{0f,0f,0f},				//Translation field
												{10.0f, 2f+0.218f/2f, -30.5f}};	//Translation soccer
		short GROUND_FLAG = 1 << 8;
		short OBJECT_FLAG = 1 << 9;
		short ALL_FLAG = -1;
		short[] collisionFlags = new short[]{GROUND_FLAG,OBJECT_FLAG};
		short[] collisionFilterFlags = new short[]{ALL_FLAG,0};
		int[] collisionObjectFlags = new int[]{CollisionFlags.KINEMATIC_OBJECT,
		CollisionFlags.CUSTOM_MATERIAL_CALLBACK
			};
													
		collisionShapes.add(groundShape);
		collisionShapes.add(ballShape);

		/*
		Transform groundTransform = new Transform();
		groundTransform.setIdentity();
		groundTransform.origin.set(new Vector3f(0.f, -56.f, 0.f));
		*/
		
		//Add collisionshapes here
		
		for (int i = 0; i<collisionShapes.size();++i) {
			
			Vector3f localInertia = new Vector3f();
			if (objectMasses[i] > 0f){
				collisionShapes.get(i).calculateLocalInertia(objectMasses[i], localInertia);
			}else{
				localInertia.set(0, 0, 0);
			}
			Transform transform = new Transform();
			transform.setIdentity();
			
			transform.origin.set(modelTransforms[i][0], modelTransforms[i][1], modelTransforms[i][2]);
			DefaultMotionState motionState = new DefaultMotionState(transform);
			
			RigidBodyConstructionInfo constInfo = new RigidBodyConstructionInfo(
					objectMasses[i], motionState, collisionShapes.get(i), localInertia);
			RigidBody rb = new RigidBody(constInfo);
			
			//rb.proceedToTransform(transform);	//Motionstate construction transform seems to have no effect
			rb.setCollisionFlags(rb.getCollisionFlags() | collisionObjectFlags[i]);
			
			if (i==1){
				rb.setFriction(0.4f);
				rb.setRestitution(0.4f);
				rb.setDamping(0.0001f,0.0001f);
				//set ball initial velocity and angular velocity
					rb.setLinearVelocity(new Vector3f(0f,(float) (25d*Math.sin(20d/180d*Math.PI)),(float) (-25d*Math.cos(20d/180d*Math.PI))));
					rb.setAngularVelocity(new Vector3f(0f,8f*2f*((float) Math.PI),0f));
				
			}else{
				rb.setFriction(0.8f);
				rb.setRestitution(1.0f);
				rb.setActivationState(CollisionObject.DISABLE_DEACTIVATION);	//Kinematic body do not deactivate
			}

			//rb.setContactCallbackFlag(collisionFlags[i]);
			//rb.setContactCallbackFilter(collisionFilterFlags[i]);
			dynamicsWorld.addRigidBody(rb);
			if (i == 1){
				dynamicsWorld.addAction(new AirDynamics(rb));
			}
		}
		
		
		

		// Do some simulation
		for (int i=0; i<120; i++) {
			dynamicsWorld.stepSimulation(1.f / 60.f, 10);

			// print ball position
			for (int j=dynamicsWorld.getNumCollisionObjects()-1; j>0; j--)
			{
				CollisionObject obj = dynamicsWorld.getCollisionObjectArray().getQuick(j);
				RigidBody body = RigidBody.upcast(obj);
				if (body != null && body.getMotionState() != null) {
					Transform trans = new Transform();
					body.getMotionState().getWorldTransform(trans);
					System.out.printf("world pos = %f,%f,%f\n", trans.origin.x,
							trans.origin.y, trans.origin.z);
				}
			}
		}
	}
}
