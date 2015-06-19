using System;
using System.Collections.Generic;
using USC.GISResearchLab.ShortestPath.GraphStructure;

namespace USC.GISResearchLab.ShortestPath.Search
{
  public class Dijkstra
  {
    Node[] previous;
    BinaryHeap Q;
    Graph g;

    public Dijkstra(Graph graph, int srcid)
    {
      this.g = graph;
      graph.getNode(srcid).myDynamicData.G = 0;
      Q = new BinaryHeap(g);
      Q.BuildHeap();
      previous = new Node[g.NodeCount];
    }
    public Node[] PTHS
    {
      get { return previous; }
    }
    public Node[] findPath()
    {
      Node current;
      float alt;
      Neighbor[] nbs;
      while (!Q.isEmpty())
      {
        current = Q.ExtractMin();
        nbs = current.Neighbors.toArray();
        for (int i = 0; i < nbs.Length; i++)
        {
          alt = current.myDynamicData.G + nbs[i].GetCost(Graph.metricType);
          if (alt < g.getNode(nbs[i].ID).myDynamicData.G)
          {
            Q.DecreaseKey(g.getNode(nbs[i].ID), alt);
            previous[nbs[i].ID] = current;
          }
        }
      }
      return previous;
    }
  }
}