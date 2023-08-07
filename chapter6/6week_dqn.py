import gym
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import random
import time

# DQN 학습 모델 정의
class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, 24)
        self.fc2 = nn.Linear(24, 24)
        self.fc3 = nn.Linear(24, action_size)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

# DQN 에이전트 정의
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = []
        self.batch_size = 32
        self.gamma = 0.99
        self.epsilon = 1.0
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01
        self.model = DQN(state_size, action_size)
        self.optimizer = optim.Adam(self.model.parameters(), lr=0.001)

        self.target_model = DQN(state_size, action_size)
        self.target_model.load_state_dict(self.model.state_dict())
        # 일정 주기로 타겟 네트워크 업데이트
        self.target_update_frequency = 100

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if random.random() <= self.epsilon:
            return random.randrange(self.action_size)
        state = torch.tensor(state, dtype=torch.float).unsqueeze(0)
        q_values = self.model(state)
        return torch.argmax(q_values).item()

    def replay(self):
        if len(self.memory) < self.batch_size:
            return
        batch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        states = torch.tensor(states, dtype=torch.float)
        actions = torch.tensor(actions, dtype=torch.long).view(-1, 1)
        rewards = torch.tensor(rewards, dtype=torch.float).view(-1, 1)
        next_states = torch.tensor(next_states, dtype=torch.float)
        dones = torch.tensor(dones, dtype=torch.float).view(-1, 1)

        self.optimizer.zero_grad()
        current_q_values = self.model(states).gather(1, actions)
        next_q_values = self.target_model(next_states).max(1)[0].unsqueeze(1)
        target_q_values = rewards + (1 - dones) * self.gamma * next_q_values
        loss = F.mse_loss(current_q_values, target_q_values.detach())
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
        # 주기적으로 타겟 네트워크 업데이트
        if episode % self.target_update_frequency == 0:
            self.target_model.load_state_dict(self.model.state_dict())

# 환경 생성
env = gym.make('CartPole-v1')
state_size = env.observation_space.shape[0]
action_size = env.action_space.n

# DQN 에이전트 생성
agent = DQNAgent(state_size, action_size)

# 학습
num_episodes = 3500
for episode in range(num_episodes):
    state = env.reset()
    done = False
    score = 0

    while not done:
        action = agent.act(state)
        next_state, reward, done, _ = env.step(action)
        agent.remember(state, action, reward, next_state, done)
        state = next_state
        score += reward

    agent.replay()

    if episode % 300 == 0:
        print("Episode: {}, Score: {}".format(episode+1, score))

# 학습된 모델 테스트
num_test_episodes = 10
for episode in range(num_test_episodes):
    state = env.reset()
    done = False
    score = 0

    while not done:
        action = agent.act(state)
        state, reward, done, _ = env.step(action)
        score += reward

    print("Test Episode: {}, Score: {}".format(episode+1, score))
    

done = False
state = env.reset()
score = 0
while not done:
    action = agent.act(state)
    env.render()
    time.sleep(0.1)
    state, rew, done, _ = env.step(action)
    score += rew 
