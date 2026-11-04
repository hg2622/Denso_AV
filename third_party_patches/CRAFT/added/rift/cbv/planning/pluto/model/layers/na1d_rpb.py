# Drop-in NeighborhoodAttention1D with relative positional bias (rpb), matching natten <= 0.17 semantics.
# natten >= 0.20 removed rel_pos_bias, but the released Pluto checkpoints contain trained `attn.rpb` tensors of shape
# (num_heads, 2*kernel_size-1). Dropping them would silently change the model, so the layer is implemented here with
# explicit sliding-window attention: exact, if slower than the fused kernel (sequence length here is only ~21 steps).
import torch
import torch.nn as nn
import torch.nn.functional as F


class NeighborhoodAttention1DRPB(nn.Module):
    def __init__(self, dim, num_heads, kernel_size, dilation=1, qkv_bias=True, qk_scale=None,
                 attn_drop=0.0, proj_drop=0.0, **kwargs):
        super().__init__()
        assert kernel_size % 2 == 1, "kernel_size must be odd"
        self.num_heads = num_heads
        self.head_dim = dim // num_heads
        self.scale = qk_scale or self.head_dim ** -0.5
        self.kernel_size = kernel_size
        self.dilation = 1 if dilation is None else max(1, int(dilation))   # natten treats None as 1
        self.qkv = nn.Linear(dim, dim * 3, bias=qkv_bias)
        self.rpb = nn.Parameter(torch.zeros(num_heads, 2 * kernel_size - 1))
        self.attn_drop = nn.Dropout(attn_drop)
        self.proj = nn.Linear(dim, dim)
        self.proj_drop = nn.Dropout(proj_drop)

    def forward(self, x):
        B, L, C = x.shape
        k, d = self.kernel_size, self.dilation
        half = k // 2
        qkv = self.qkv(x).reshape(B, L, 3, self.num_heads, self.head_dim).permute(2, 0, 3, 1, 4)
        q, kk, v = qkv[0] * self.scale, qkv[1], qkv[2]            # each (B, H, L, hd)

        # natten pins the window inside the sequence: near the borders the window shifts instead of being truncated,
        # so every token attends to exactly k neighbours and the bias index is taken relative to that shifted window.
        span = d * (k - 1)
        centre = torch.arange(L, device=x.device).clamp(half * d, max(L - 1 - half * d, half * d))
        start = (centre - half * d).clamp(0, max(L - 1 - span, 0))
        idx = start[:, None] + d * torch.arange(k, device=x.device)[None, :]   # (L, k)
        idx = idx.clamp(max=L - 1)

        k_n = kk[:, :, idx, :]                                    # (B, H, L, k, hd)
        v_n = v[:, :, idx, :]
        attn = (q.unsqueeze(3) * k_n).sum(-1)                     # (B, H, L, k)

        pos = idx - torch.arange(L, device=x.device)[:, None]     # relative offset of each neighbour
        bias_idx = (pos // d) + (k - 1)                           # -> [0, 2k-2]
        attn = attn + self.rpb[:, bias_idx.clamp(0, 2 * k - 2)].unsqueeze(0)

        attn = self.attn_drop(attn.softmax(dim=-1))
        out = (attn.unsqueeze(-1) * v_n).sum(3)                   # (B, H, L, hd)
        out = out.permute(0, 2, 1, 3).reshape(B, L, C)
        return self.proj_drop(self.proj(out))
