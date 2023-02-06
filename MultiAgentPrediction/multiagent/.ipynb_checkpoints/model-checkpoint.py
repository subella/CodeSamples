import torch
import torch.nn as nn
import torch.nn.functional as F
import torch_geometric.transforms as T
from torch_geometric.nn import EdgeConv, global_mean_pool
from torch.nn import Sequential as Seq, Linear as Lin, ReLU, BatchNorm1d
from torch_scatter import scatter_mean
from torch_geometric.nn import MetaLayer

inputs = 128
hidden = 128
outputs = 3

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# +
class PastStateEncoder(nn.Module):
    def __init__(self, output_size, timesteps, features):
      super().__init__()
      self.output_size = output_size
      self.embedding = nn.Embedding(timesteps, features)
      self.conv = nn.Conv1d(features*2,output_size,3,1,1)

    def forward(self, x):
      B, L, H = x.shape
      pos = torch.arange(L).to(device)
      x = torch.cat([x, self.embedding(pos).view(1, L, 4).expand(B, L, 4)], dim=-1)
      x = x.transpose(2,1)
      return self.conv(x).mean(dim=-1)

class FusionLayer(nn.Module):
    def __init__(self):
        super().__init__()
        self.fuse = nn.Linear(129 , 128)

    def forward(self, x):
      return self.fuse(x)


# -

class EdgeModel(torch.nn.Module):
    def __init__(self,num_features_in,edge_features_in, hidden):
        super().__init__()
        self.edge_mlp = nn.Sequential(nn.Linear(2 * num_features_in + edge_features_in , hidden),
                       nn.ReLU(),
                       nn.Linear(hidden, 128))

    def forward(self, src, dest, edge_attr, u, batch):
        # src, dest: [E, F_x], where E is the number of edges.
        # edge_attr: [E, F_e]
        # u: [B, F_u], where B is the number of graphs.
        # batch: [E] with max entry B - 1.
        # print(src.shape, "THIS IS SRC")
        # print(dest.shape, "THIS IS dest")
        # print(edge_attr.shape, "THIS IS edge_attr")
        out = torch.cat([src, dest, edge_attr], 1)
        out = self.edge_mlp(out)
        return out

class NodeModel(torch.nn.Module):
    def __init__(self,num_features_in, edge_features_in, hidden):
        super().__init__()
        self.node_mlp_1 = nn.Sequential(nn.Linear(2*num_features_in, hidden),        
                       nn.ReLU(),
                       nn.Linear(hidden, 128))

    def forward(self, x, edge_index, edge_attr, u, batch):
        # x: [N, F_x], where N is the number of nodes.
        # edge_index: [2, E] with max entry N - 1.
        # edge_attr: [E, F_e]
        # u: [B, F_u]
        # batch: [N] with max entry B - 1.
        row, col = edge_index
        out = torch.cat([x[row], edge_attr], dim=1)
        return self.node_mlp_1(out) 

def make_meta(num_features_in,edge_features_in,hidden):
  return MetaLayer(edge_model=EdgeModel(num_features_in,edge_features_in,hidden), 
                   node_model=NodeModel(num_features_in,edge_features_in, hidden))


class GraphNeuralNetwork(torch.nn.Module):
    def __init__(self):
        super(GraphNeuralNetwork, self).__init__()
        self.final_mlp = Seq(Lin(hidden, hidden), 
                            BatchNorm1d(hidden),
                            ReLU(),
                            Lin(hidden, outputs))
        self.rnn = PastStateEncoder(128, 5, 4)
        self.fuse = FusionLayer()
        self.gnn1 = make_meta(128,4,128)
        self.gnn2 = make_meta(128,128,128)
        
    def forward(self, x, edge_index, edge_attr, batch, node_type):
        x = self.rnn(x)
        x = torch.cat([x,node_type], dim=-1)
        x = self.fuse(x)
        x, edge_attr, u = self.gnn1(x, edge_index, edge_attr, None, batch)
        x, edge_attr, u = self.gnn2(x, edge_index, edge_attr, None, batch)
        out = self.final_mlp(edge_attr)
        return out
