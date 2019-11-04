using System;
using System.Collections.Generic;
using System.Threading;
using USC.GISResearchLab.ShortestPath.GraphStructure;

namespace USC.GISResearchLab.ShortestPath.Search
{
    public class AStar
    {
        #region Properties

        private uint MaxMiles2Go;
        private uint MaxHours2Go;
        BinaryHeap heap;
        MultiDictionary<long, long, Node> closedList;
        Graph myGraph;

        #endregion

        public AStar(Graph graph, uint maxDist, uint maxHour)
        {
            myGraph = graph;
            MaxMiles2Go = maxDist;
            MaxHours2Go = maxHour;
        }

        public double Heuristic(Node curr, Node dest, Graph.MetricType metric)
        {
            //initialize the heuristic as great circle distance
            double h;
            try
            {
                h = AStarUtils.GCDistance(curr, dest);
                if (metric == Graph.MetricType.Time) h = h / 75.0;
            }
            catch (ThreadAbortException te)
            {
                throw te;
            }
            return h;
        }

        #region Landmark related methods
        /*
		public void LMSelection(Node curr_src, Node curr_dst)
		{
			double srcH, dstH, max = 0;

			foreach (Node lm in landMarkList)
			{
				srcH = this.LM2NodeShortestPthDist(curr_src, lm);
				dstH = this.LM2NodeShortestPthDist(curr_dst, lm);
				if (srcH - dstH > max)
				{
					max = srcH - dstH;
					bestLM = lm;
				}
			}
		}

		public double LM2NodeShortestPthDist(Node aNode, Node aLandMark)
		{
			double dist = 0.0;
			Hashtable ht = ((Hashtable)precomputes[aNode]);
			if (ht == null)
				dist = AStarUtils.GCDistance(aNode, aLandMark);
			else
			{
				object obj = ht[aLandMark];
				if (obj == null)
					dist = AStarUtils.GCDistance(aNode, aLandMark);
				else
					dist = (double)(obj);
			}
			return dist;
		}
		
		public double bestUpperBound(AStarInput input)
		{
			double bestBound = double.MaxValue, currVal;
			if (landMarkList != null)
			{
				foreach (Node lm in landMarkList)
				{
					currVal = LM2NodeShortestPthDist(input.From, lm) + LM2NodeShortestPthDist(input.To, lm);
					if (currVal < bestBound)
					{
						bestBound = currVal;
					}
				}
			}
			return bestBound;
		}
		*/
        #endregion

        /**
		 * this function is called by the website, specifically by: shortestpathEngine in the Core.Runner folder
		 * @param: input contains the source and destination node
		 * return: shortest path in terms of distance or time
		 */
        public AStarOutput CalculatePath(ref AStarInput input)
        {
            var ret = new AStarOutput();
            heap = null;
            closedList = null;

            PseudoEdge extract = null;
            Node current = null, neighbor = null, FinalDest = null;
            GraphNode gNeighbor = null;
            double alternativeG = 0.0, possibleToG = Node.Cost_NoRoute, UpperF = MaxHours2Go, minWDistance = 0.0;
            byte i = 0;
            bool SearchIsFinished = false;
            Dictionary<long, double> ToNN = null;
            Dictionary<Node, double> DiscoveredDestinations = null;
            Dictionary<long, long> FromNodesPrev = null;

            try
            {
                input.To.InpNode.PreviousNode = null;
                input.To.InpNode.PreviousEdge = 100;
                input.To.InpNode.G = Node.Cost_NoRoute;

                if (input.Metric == Graph.MetricType.Distance) UpperF = MaxMiles2Go;
                else UpperF = MaxHours2Go;

                // this is the smart way of setting the upper limit to help the algorithm terminate faster on inapopriate queries.
                // Of course this kind of termination violates the accuracy and should be used with care.
                if (UpperF <= 0) UpperF = 20.0 * Heuristic(input.From.InpNode, input.To.InpNode, input.Metric);
                if (input.Metric == Graph.MetricType.Time) UpperF = UpperF * 4.0;

                if ((input.From == null) || (input.To == null)) return new AStarOutput(Node.Cost_NullInput);
                if ((input.From.NearestNodes == null) || (input.To.NearestNodes == null) ||
                    (input.From.NearestNodes.Count == 0) || (input.To.NearestNodes.Count == 0))
                    return new AStarOutput(Node.Cost_NoNearNode);

                foreach (var l in input.From.NearestNodes.Values)
                    foreach (var n in l)
                        if (myGraph.GetNode(n.UID) == null)
                            throw new NullReferenceException("The Astar input source node is not in the graph.");

                foreach (var l in input.To.NearestNodes.Values)
                    foreach (var n in l)
                        if (myGraph.GetNode(n.UID) == null)
                            throw new NullReferenceException("The Astar input destination node is not in the graph.");

                try
                {
                    // proccessing possible destinations
                    // int heapCap = Convert.ToInt32(from1.H * 500.0);
                    // if (input.Metric == Graph.MetricType.Time) heapCap = heapCap * 2;
                    ToNN = new Dictionary<long, double>(input.To.NearestNodes.Count);
                    heap = new BinaryHeap(1000);
                    closedList = new MultiDictionary<long, long, Node>(10000);

                    foreach (var l in input.To.NearestNodes)
                        foreach (var n in l.Value)
                        {
                            if (ToNN.ContainsKey(n.UID))
                            {
                                if (ToNN[n.UID] > l.Key) ToNN[n.UID] = l.Key;
                            }
                            else
                            {
                                ToNN.Add(n.UID, l.Key);
                            }
                        }
                    DiscoveredDestinations = new Dictionary<Node, double>(ToNN.Count);
                    FromNodesPrev = new Dictionary<long, long>(input.From.NearestNodes.Count);

                    // proccessing possible start points and prepare the heap
                    foreach (var l in input.From.NearestNodes)
                    {
                        ret.NearNodeDistance = l.Key;
                        foreach (var n in l.Value)
                        {
                            current = new Node(myGraph.GetNode(n.UID));
                            current.PreviousNode = input.From.InpNode;
                            if (input.Metric == Graph.MetricType.Distance) current.G = n.G;
                            else current.G = n.H;

                            current.H = Heuristic(current, input.To.InpNode, input.Metric);

                            // insert 'current' into the empty heap
                            if (!heap.Contains(n.PreviousNode.PreviousNode.UID, current) &&
                                !closedList.Contains(n.PreviousNode.PreviousNode.UID, current.UID))
                            {
                                heap.Insert(n.PreviousNode.PreviousNode.UID, current);
                                if (FromNodesPrev.ContainsKey(current.UID))
                                {
                                    FromNodesPrev[current.UID] = n.PreviousNode.PreviousNode.UID;
                                }
                                else
                                {
                                    FromNodesPrev.Add(current.UID, n.PreviousNode.PreviousNode.UID);
                                }
                            }
                        }

                        // start the A* with the prepared heap until heap gets empty or some destinations are found

                        while (!heap.isEmpty())
                        {
                            extract = heap.ExtractMin();
                            current = extract.To;

                            // add newly discovered edge to the 'closed list'
                            if (closedList.Contains(extract.PrevUID, current.UID)) throw new Exception("Closed/Open List violation " + current + ".");
                            else closedList.Set(extract.PrevUID, current.UID, current);

                            // termination condition
                            if (ToNN.ContainsKey(current.UID))
                            {
                                foreach (var n in input.To.NearestNodes[ToNN[current.UID]])
                                {
                                    if (n.Equals(current))
                                    {
                                        if (input.Metric == Graph.MetricType.Distance) current.H = n.G;
                                        else current.H = n.H;
                                        break;
                                    }
                                }
                                if (!(DiscoveredDestinations.ContainsKey(current))) DiscoveredDestinations.Add(current, ToNN[current.UID]);
                                SearchIsFinished = true;

                                // a control system to avoid going too far to discover aditional destinations
                                if (DiscoveredDestinations.Count == 1)
                                {
                                    UpperF = current.F;
                                    if (input.Metric == Graph.MetricType.Distance) UpperF += 3.1;
                                    else UpperF += 1.0;
                                }
                            }

                            if (DiscoveredDestinations.Count == ToNN.Count) break;

                            for (i = 0; i < current.NeighborsCount; i++)
                            {
                                gNeighbor = myGraph.GetNode(current.Neighbors[i].DestinationNodeID);
                                // if edge already expanded, skip it
                                if (closedList.Contains(current.UID, gNeighbor.UID)) continue;

                                // Manoeuvre Restriction. Do restriction test only when it's not the source node.            
                                if (current.Neighbors[i].ContainsRestriction(extract.PrevUID))
                                {
                                    ret.ManoeuvreCount++;
                                    continue;
                                }

                                alternativeG = current.G + current.Neighbors[i].GetCost(input.Metric);
                                neighbor = heap.FindGraphNode(current.UID, gNeighbor);

                                if (neighbor == null)
                                {
                                    neighbor = new Node(gNeighbor);
                                    neighbor.G = alternativeG;
                                    neighbor.PreviousNode = current;
                                    neighbor.PreviousEdge = i;
                                    neighbor.H = Heuristic(neighbor, input.To.InpNode, input.Metric);

                                    // Don't go too Far
                                    if (neighbor.F < UpperF) heap.Insert(current.UID, neighbor);
                                }
                                else if (alternativeG < neighbor.G)
                                {
                                    heap.DecreaseKey(current.UID, neighbor, alternativeG);
                                    neighbor.PreviousNode = current;
                                    neighbor.PreviousEdge = i;
                                }
                            }
                        } // end while

                        if (SearchIsFinished) break;
                    }
                }
                catch (ThreadAbortException te)
                {
                    throw te;
                }
                catch (Exception ex)
                {
                    throw new Exception("There was an error preparing Astar inputs.", ex);
                }

                // determining the final answer
                ret.Cost = Node.Cost_NoRoute;

                if (DiscoveredDestinations.Count > 0)
                {
                    foreach (var d in DiscoveredDestinations)
                    {
                        if (FinalDest == null)
                        {
                            FinalDest = d.Key;
                            minWDistance = d.Value;
                        }
                        else
                        {
                            if (d.Value < minWDistance)
                            {
                                FinalDest = d.Key;
                                minWDistance = d.Value;
                            }
                            else if ((d.Value == minWDistance) && (d.Key.F < FinalDest.F))
                            {
                                FinalDest = d.Key;
                                minWDistance = d.Value;
                            }
                        }
                    }

                    ret.NearNodeDistance += minWDistance;
                    ret.Cost = FinalDest.F;
                    input.To.InpNode.PreviousNode = FinalDest;
                    input.To.InpNode.G = ret.Cost;

                    // other cost computation
                    if (ret.Cost >= 0.0) ret.OtherCost = ComputeOtherCost(input, FinalDest, ToNN);
                }
            }
            catch (ThreadAbortException te)
            {
                throw te;
            }
            catch (Exception e)
            {
                ret.Cost = Node.Cost_LogicError;
                ret.ErrorMessage = e.Message;
                throw new Exception("Error in AStar Calculation", e);
            }
            finally
            {
                if (heap != null) ret.OpenNodesCount = heap.Count;
                if (closedList != null) ret.ClosedNodesCount = closedList.Count;
                heap = null;
                closedList = null;
            }

            return ret;
        }

        /**
         *this function is specially designed for computing the cost as distance or time
         *after the searching is finished
         */
        private static double ComputeOtherCost(AStarInput inp, Node FinalDest, Dictionary<long, double> ToNN)
        {
            double cost = -1.0, sum = 0.0;
            try
            {
                bool OperationFinished = false;
                int i = 0;
                Node cr = null;
                Graph.MetricType metric = Graph.MetricType.Distance;
                if (inp.Metric == Graph.MetricType.Distance) metric = Graph.MetricType.Time;

                // adding the 'to' side other cost
                foreach (var n in inp.To.NearestNodes[ToNN[FinalDest.UID]])
                    if (n.Equals(FinalDest))
                    {
                        if (metric == Graph.MetricType.Distance) sum += n.G;
                        else sum += n.H;
                        break;
                    }

                for (cr = FinalDest; !cr.PreviousNode.Equals(inp.From.InpNode); cr = cr.PreviousNode)
                {
                    cost = double.MaxValue;
                    if (cr.PreviousEdge < 100)
                    {
                        cost = cr.PreviousNode.Neighbors[cr.PreviousEdge].GetCost(metric);
                    }
                    else
                    {
                        for (i = 0; i < cr.PreviousNode.NeighborsCount; i++)
                            if (cr.PreviousNode.Neighbors[i].DestinationNodeID == cr.UID)
                            {
                                cost = cr.PreviousNode.Neighbors[i].GetCost(metric);
                                break;
                            }
                    }
                    if (cost == double.MaxValue) throw new Exception("Broken link between nodes during cost computation.");
                    else sum += cost;
                }

                // adding the 'from' side other cost
                foreach (var l in inp.From.NearestNodes)
                {
                    foreach (var n in l.Value)
                    {
                        if (n.Equals(cr))
                        {
                            if (metric == Graph.MetricType.Distance) sum += n.G;
                            else sum += n.H;
                            OperationFinished = true;
                        }
                        if (OperationFinished) break;
                    }
                    if (OperationFinished) break;
                }
            }
            catch (ThreadAbortException te)
            {
                throw te;
            }
            return sum;
        }

        public static LinkedList<Node> GetPath(AStarInput inp)
        {
            var path = new LinkedList<Node>();
            try
            {
                path.AddFirst(inp.To.InpNode);
                bool OperationFinished = false;
                Node ToNN = null, cur = null;

                // finding the mid point between 'ToNN' and 'To' node
                foreach (var l in inp.To.NearestNodes)
                {
                    foreach (var n in l.Value)
                    {
                        if (n.Equals(inp.To.InpNode.PreviousNode))
                        {
                            ToNN = inp.To.InpNode.PreviousNode;
                            path.AddFirst(n.PreviousNode);
                            OperationFinished = true;
                            break;
                        }
                    }
                    if (OperationFinished) break;
                }
                OperationFinished = false;

                // traversing the mid graph nodes backward to the 'From' node
                for (cur = ToNN; !cur.PreviousNode.Equals(inp.From.InpNode); cur = cur.PreviousNode)
                    path.AddFirst(cur);

                path.AddFirst(cur);

                // finding the mid point between 'FromNN' and 'From' node
                foreach (var l in inp.From.NearestNodes)
                {
                    foreach (var n in l.Value)
                    {
                        if (n.Equals(cur))
                        {
                            path.AddFirst(n.PreviousNode);
                            OperationFinished = true;
                            break;
                        }
                    }
                    if (OperationFinished) break;
                }

                path.AddFirst(inp.From.InpNode);
            }
            catch (ThreadAbortException te)
            {
                throw te;
            }
            return path;
        }

        public static string ComputeKMLPath(LinkedList<Node> path)
        {
            string ret;
            try
            {
                if ((path != null) && (path.Count > 0)) ret = (new KMLLine(path)).ToKMLString();
                else ret = string.Empty;
            }
            catch (ThreadAbortException te)
            {
                throw te;
            }
            return ret;
        }
    }
}
