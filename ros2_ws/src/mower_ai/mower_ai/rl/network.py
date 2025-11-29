import torch
import torch.nn as nn
import torch.nn.functional as F

class ResidualBlock(nn.Module):
    def __init__(self, input_dim, output_dim):
        super().__init__()
        self.linear = nn.Linear(input_dim, output_dim)
        self.ln = nn.LayerNorm(output_dim)

    def forward(self, x):
        return torch.tanh(self.ln(self.linear(x)))

class Backbone(nn.Module):
    def __init__(self, obs_dim):
        super().__init__()
        # 로그 분석 결과 반영:
        # 1. (입력 100 -> 은닉 512)
        # 2. (은닉 512 -> 은닉 512)
        # 3. (은닉 512 -> 은닉 256)
        self.hidden_layers = nn.ModuleList([
            ResidualBlock(obs_dim, 512),  # [수정] 256 -> 512
            ResidualBlock(512, 512),      # [수정] 256 -> 512
            ResidualBlock(512, 256)       # [수정] 입력 512, 출력 256
        ])
        
        # 4. (은닉 256 -> 특징 384)
        self.fc3 = nn.Linear(256, 384)

    def forward(self, x):
        for layer in self.hidden_layers:
            x = layer(x)
        return torch.tanh(self.fc3(x))

class ActorCritic(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        
        self.backbone = Backbone(obs_dim)
        
        # Actor와 Critic은 384 차원의 특징을 입력받음
        self.actor = nn.Linear(384, act_dim)
        self.critic = nn.Linear(384, 1)

    def forward(self, x):
        features = self.backbone(x)
        logits = self.actor(features)
        value = self.critic(features)
        return logits, value

    def act(self, x):
        features = self.backbone(x)
        logits = self.actor(features)
        return logits