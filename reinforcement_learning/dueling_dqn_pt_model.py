#!usr/bin/env python3

import torch
import rclpy
from rclpy.node import Node
import numpy as np
import os
import time 
from datetime import datetime
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from std_msgs.msg import Float32MultiArray
from src.dueling_dqn_env import Env
from dueling_dqn_agent import *
from setup_config import *
import csv
from nav_msgs.msg import Odometry
import threading
from rclpy.executors import MultiThreadedExecutor
from torch.utils.tensorboard import SummaryWriter


dirPath = os.path.dirname(os.path.realpath(__file__))
LOG_DATA_DIR = dirPath + '/log_data'
EPISODES = num_episodes    

        
def Training(ros_handler):
    writer = SummaryWriter(log_dir=LOG_DATA_DIR + '/runs')
    mode = "train"
    load_episodes = 0
    #rospy.init_node('dueling_dqn_gazebo_pt_model')
    #pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
    #pub_get_action = rospy.Publisher('get_action', Float32MultiArray, queue_size=5)
    
    result = Float32MultiArray()
    get_action = Float32MultiArray()

    # state_size = 182
    state_size = 1
    action_size = 5
    
    # define the environment
    env = Env(action_size, ros_handler)

    # define the agent
    agent = DuelingQAgent(state_size, action_size, mode, load_episodes, ros_handler)
    episodes = []
    steps_per_episode = []
    total_rewards = []
    global_step = 0
    goal_counters_array = []
    
    # Init log files
    log_sim_info = open(LOG_DATA_DIR+'/LogInfo.txt','w+')
    
    # Date / Time
    start_time = time.time()
    now_start = datetime.now()
    dt_string_start = now_start.strftime("%d/%m/%Y %H:%M:%S")

    # Log date to files
    text = '\r\n' + '****************************************************************\n'
    text = text + 'SIMULATION START ==> ' + dt_string_start + '\r\n'
    text = text + 'INITIAL ROBOT POSITION = ( %.2f , %.2f , %.2f ) \r\n' % (env.init_x, env.init_y, 0.0)
    text = text + '****************************************************************\n'
    print(text)
    log_sim_info.write(text)
    position_log_file = open(LOG_DATA_DIR + '/robot_positions.csv', 'w', newline='') #NEWx3
    position_writer = csv.writer(position_log_file)
    position_writer.writerow(["Episode", "Step", "X", "Y", "Theta"])  # Ghi header

    for e in range(agent.load_episode + 1, EPISODES):
        text = '\r\n' + '_____ EPISODE: ' + str(e) + ' _____' + '\r\n'
        text = text + '----------------------------------------------------------------\n'
        print(text)
        done = False
        counters = 0
        state = env.reset()
        reward_per_episode = 0
        #print("Shape after reset:", state.shape)
        if len(state.shape)==2:
            for step in range(agent.episode_step):
                #print(f"Episode: {e} - Step: {step}", end='\r')
                
                action = agent.getAction(state)
                
                next_state, reward, done, counters = env.step(action)
               
                odom = ros_handler.odom
                tf = ros_handler.tf
                x, y, theta = env.getOdometry(odom, tf) # Giả sử hàm này trả về vị trí hiện tại của robot
                position_writer.writerow([e, step, x, y, theta]) # NEWx2 
                done = np.bool_(done)
               
                agent.RAM.add(state, action, reward, next_state, done)
                
                if agent.RAM.len >= agent.train_start:
                    if global_step % 1 == 0:
                        
                        agent.TrainModel()

                reward_per_episode += reward
                state = next_state
                get_action.data = [float(action), float(reward_per_episode), float(reward)]
                ros_handler.pub_get_action.publish(get_action)

                if e >=1000 and e%100==0:
                    torch.save(agent.Pred_model.state_dict(), agent.dirPath + str(e) + '.pt')

                if step >= 1000:
                    print('\n==> Time out! Maxed step per episode\n')
                    done = True
                

                if done:
                    # Login tensorboard
                    writer.add_scalar('Train/Reward', reward_per_episode, e)
                    writer.add_scalar('Train/Epsilon', agent.epsilon, e)
                    writer.add_scalar('Train/Steps', step, e)
                    writer.add_scalar('Train/Goal_Counter', counters, e)

                    result.data = [float(reward_per_episode), float(action)]
                    ros_handler.pub_result.publish(result)
                    agent.updateTargetModel()
                    total_rewards.append(reward_per_episode)
                    steps_per_episode.append(step)
                    goal_counters_array.append(counters)
                    episodes.append(e)
                    m, s = divmod(int(time.time() - start_time), 60)
                    h, m = divmod(m, 60)

                    ros_handler.get_logger().info(f'Episode: {e} , Reward per episode: {reward_per_episode:.2f} , Memory: {agent.RAM.len} , Epsilon: {agent.epsilon:.4f} , Time: {h}:{m:02d}:{s:02d}')
                    text = text + 'Episode: %d , Reward per episode: %.2f , Memory: %d , Epsilon: %.4f , Time: %d:%02d:%02d \r\n'%\
                        (e, reward_per_episode, agent.RAM.len, agent.epsilon, h, m, s)
                    text = text + '----------------------------------------------------------------\r\n'
                    log_sim_info.write('\r\n'+text)
                    
                    param_keys = ['epsilon']
                    param_values = [agent.epsilon]
                    param_dictionary = dict(zip(param_keys, param_values))
                    break

                global_step += 1    # step counting when the robot takes action for each iteration
                if global_step % agent.target_update == 0:
                    ros_handler.get_logger().info("UPDATE TARGET NETWORK")
                    agent.updateTargetModel()
                

        if agent.epsilon > agent.epsilon_min:
            agent.epsilon *= agent.epsilon_decay
        
        # Save data to directory
        np.savetxt(LOG_DATA_DIR + '/steps_per_episode.csv', steps_per_episode, delimiter = ' , ')
        np.savetxt(LOG_DATA_DIR + '/reward_per_episode.csv', total_rewards, delimiter = ' , ')
        np.savetxt(LOG_DATA_DIR + '/goal_counters_per_episode.csv', goal_counters_array, delimiter = ' , ')
    
    # Close the log file
    log_sim_info.close()
    position_log_file.close() # Newx1
    writer.close()    
        
if __name__ == '__main__':
    
    rclpy.init()

    ros_handler = RosHandler()

    executor = MultiThreadedExecutor()
    executor.add_node(ros_handler)

    
    training_thread = threading.Thread(target=Training, args=(ros_handler,))
    training_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ros_handler.destroy_node()
        rclpy.shutdown()
        print("Simulation terminated.")