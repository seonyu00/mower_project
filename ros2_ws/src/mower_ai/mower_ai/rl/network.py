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
        self.hidden_layers = nn.ModuleList([
            ResidualBlock(obs_dim, 256),  
            ResidualBlock(256, 256),      
            ResidualBlock(256, 128)       
        ])
        
        # 4. (은닉 256 -> 특징 384)
        self.fc3 = nn.Linear(128, 256)

    def forward(self, x):
        for layer in self.hidden_layers:
            x = layer(x)
        return torch.tanh(self.fc3(x))

class ActorCritic(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        
        self.backbone = Backbone(obs_dim)
        
        # Actor와 Critic은 384 차원의 특징을 입력받음
        self.actor = nn.Linear(256, act_dim)
        self.critic = nn.Linear(256, 1)

    def forward(self, x):
        features = self.backbone(x)
        logits = self.actor(features)
        value = self.critic(features)
        return logits, value

    def act(self, x):
        features = self.backbone(x)
        logits = self.actor(features)
        return logits