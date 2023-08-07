import numpy as np 
import gym
import time



# 입실론 그리디
def es_greed(Q, state, es = 0.1):
    # 랜덤으로 0~1 사이 선택된 값이 입실론값(es) 보다 작으면 랜덤으로 액션 선택
    if np.random.uniform(0,1) < es: 
        return np.random.randint(Q.shape[1])
    else:
    # 입실론값보다 크다면 Q함수에서 해당 state에서 가장 큰 value를 선택 
        return act_greedy(Q,state)
    
#그리디 액션
def act_greedy(Q, state):
    return np.argmax(Q[state])

# Q러닝 알고리즘 
def Q_learning(env, ep_num=10000, lr=0.01,  es=0.3, gamma=0.95, es_decay=0.00005):
    action_num = env.action_space.n
    state_num = env.observation_space.n
    game_rew = []
    test_rews = []

    # [state * action]으로 이루어진 Q table값 0으로 초기화    
    Q = np.zeros((state_num, action_num))
    
    for ep in range(ep_num):
        state = env.reset()
        done = False
        total_rew = 0
        
        # 입실론값이 threshold값인 0.01에 도달할 때까지 값을 줄임
        if es > 0.01:
            es -= es_decay

        while not done:
            # 입실론 그리드 action을 진행하며 action을 얻음
            action = es_greed(Q, state, es)
            
            # 얻은 action을 이용해 해당 환경에서 한 step 진행하여 이때 얻는 다음 state와 reward, 종료 여부를 변수로 저장
            next_state, rew, done, _ = env.step(action) 
            
            # 현재 state에서 취한 action값에 lr*(rew + gamma*np.max(Q[next_state]) - Q[state][action])를 더함
            # gamma*np.max(Q[next_state]) - Q[state][action]를 통해 다음 상태에서의 최대 Q값과 현재 state에서 action에 대한 Q값의 차이 계산
            # 이러한 값을 현재 얻은 rew에 더하며 Q함수가 최대 rew를 받는 쪽으로 학습 진행
            Q[state][action] = Q[state][action] + lr*(rew + gamma*np.max(Q[next_state]) - Q[state][action])

            # 다음 state를 현재 state로 바꿈 
            state = next_state
            total_rew += rew
            if done:
                game_rew.append(total_rew)

        # 300 에피소드마다 학습을 테스트하고 결과 출력
        if (ep % 500) == 0:
            test_rew = run_episodes(env, Q, 1000)
            print("learning episode epoch :{:5d}  reward:{:2.4f} epsilon:{:2.4f}  ".format(ep, test_rew, es))
            test_rews.append(test_rew)
            
    return Q

# 학습된 Qtable을 활용해 검증을 진행하기 위한 함수 
def run_episodes(env, Q, num_episodes=100, to_print=False):
    tot_rew = []
    state = env.reset()

    for _ in range(num_episodes):
        done = False
        game_rew = 0

        while not done:
            # 그리디 액션을 선택
            next_state, rew, done, _ = env.step(act_greedy(Q, state))

            state = next_state
            game_rew += rew 
            if done:
                state = env.reset()
                tot_rew.append(game_rew)

    if to_print:
        print('Mean score: %.3f of %i games!'%(np.mean(tot_rew), num_episodes))

    return np.mean(tot_rew)

# 결과 테스트를 위해 env.render()함수가 추가된 함수
def run_episode(env, Q):
    state = env.reset()
    done = False

    while not done:
        # 현재 상태에서 greedy action 선택
        action = act_greedy(Q, state)
        next_state, _, done, _ = env.step(action)
        # 현재 상태를 시각화
        env.render()  
        state = next_state
        time.sleep(1)





if __name__ == '__main__':
    env = gym.make('Taxi-v3')
    Q_qlearning = Q_learning(env, lr=.1, ep_num=10000, es=0.4, gamma=0.95, es_decay=0.001)
    run_episode(env, Q_qlearning)   