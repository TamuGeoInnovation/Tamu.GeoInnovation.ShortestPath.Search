using System;
using System.Collections.Generic;
using System.Data.SqlClient;
using USC.GISResearchLab.ShortestPath.GraphStructure;

namespace USC.GISResearchLab.ShortestPath.Search
{
    public class NearestNodeIO
    {
        public SqlConnection Con = null;
        public string GraphName = string.Empty;
        public Node InpNode = null;
        public SortedDictionary<Double, List<Node>> NearestNodes = null;
        public bool IsSource = true;
        public double MaxNearNodeDistance = 1000.0;
        public double DiscoveredRadius = 0.0;
    }

    public class AStarInput
    {
        public NearestNodeIO From;
        public NearestNodeIO To;
        public Graph.MetricType Metric;

        public AStarInput(NearestNodeIO from, NearestNodeIO to, Graph.MetricType metric)
        {
            From = from;
            To = to;
            Metric = metric;
        }
        public AStarInput() : this(null, null, Graph.MetricType.Distance) { }
    }

    public class AStarOutput
    {
        public double Cost;
        public double OtherCost;
        public int OpenNodesCount;
        public int ClosedNodesCount;
        public int ManoeuvreCount;
        public string ErrorMessage;
        public double NearNodeDistance = Node.Cost_Undeterm;

        public AStarOutput(double cost)
        {
            Cost = cost;
            OtherCost = Node.Cost_Undeterm;
            OpenNodesCount = 0;
            ClosedNodesCount = 0;
            ManoeuvreCount = 0;
            ErrorMessage = string.Empty;
        }

        public AStarOutput() :
            this(Node.Cost_Undeterm)
        { }
    }
}