using Microsoft.SqlServer.Types;
using System;
using System.Collections.Generic;
using System.Data;
using System.Data.SqlClient;
using System.Data.SqlTypes;
using USC.GISResearchLab.Common.Geographics.DistanceFunctions;
using USC.GISResearchLab.Common.Geographics.Units;
using USC.GISResearchLab.ShortestPath.GraphStructure;
using USC.GISResearchLab.ShortestPath.RoadNetworkData;

namespace USC.GISResearchLab.ShortestPath.Search
{
    public class AStarUtils
    {
        public static int MIN_NearestLevelCount = 4;

        //Convergence is a concept to measure the closeness from a given point
        //to the virtual line connecting source and destination. 
        public static double ConvergenceValidation(AStarInput input, Node curr)
        {
            //a, b, c are edges & A, B, C are angles
            double edge_a, edge_b, edge_c, angle_A, dist;
            edge_a = GreatCircleDistance.Distance_GetDeg_OutRad(curr.Latitude, curr.Longitude, input.To.InpNode.Latitude, input.To.InpNode.Longitude);
            edge_b = GreatCircleDistance.Distance_GetDeg_OutRad(curr.Latitude, curr.Longitude, input.From.InpNode.Latitude, input.From.InpNode.Longitude);
            edge_c = GreatCircleDistance.Distance_GetDeg_OutRad(input.From.InpNode.Latitude, input.From.InpNode.Longitude, input.To.InpNode.Latitude, input.To.InpNode.Longitude);
            //A=acos((cos(a) - cos(b)*cos(c))/(sin(b)*sin(c)))
            if (edge_b == 0 || edge_a == 0)
                return 0;
            else
                angle_A = Math.Acos((Math.Cos(edge_a) - Math.Cos(edge_b) * Math.Cos(edge_c)) / (Math.Sin(edge_b) * Math.Sin(edge_c)));
            //for right spherical triangle where B = 90 degree: sin(a)=sin(A)*sin(b)
            if (edge_c >= 0 && edge_b >= 0 && edge_a >= 0 && angle_A >= 0)
                dist = Math.Asin(Math.Sin(angle_A) * Math.Sin(edge_b));
            else
                throw new ArgumentException("something's wrong");
            return dist * GreatCircleDistance.EARTHRADIUSASMILES;
        }

        public static bool validate(Node curr, AStarInput input, double bound)
        {
            double temp = GCDistance(curr, input.From.InpNode) + GCDistance(curr, input.To.InpNode);
            //disable this feature now (bounding rectangle)
            double vDist = AStarUtils.ConvergenceValidation(input, curr);
            return (temp < bound && vDist < 10);
        }
        public static double GCDistance(Node n1, Node n2)
        {
            return GreatCircleDistance.LinearDistance2(n1.Latitude, n1.Longitude, n2.Latitude, n2.Longitude, LinearUnitTypes.Miles);
        }

        // This method searches the DB for nearest node to to input node and then it adds
        // the nearest intersection node as a neighbor to this virtual node and returns it (spatial search)
        public static NearestNodeIO FindNearestNode(NearestNodeIO io)
        {
            try
            {
                if (io.Con.State != ConnectionState.Open) io.Con.Open();
                io.NearestNodes = new SortedDictionary<double, List<Node>>();

                SqlCommand cmd = null;
                int count = 0, circleBufferIndex = 0;
                double nearNodeSpeed = 0.0, ft_speed = 0.0, tf_speed = 0.0, len;
                double l1 = 0.0, l2 = 0.0, p1inp = 0.0, travel = 0.0, p1p2 = 0.0,
                        dist = 10.0 * io.MaxNearNodeDistance, total = 0.0, opTotal = 0.0;
                double te1 = 0.0, te2 = 0.0, alpha = 0.0, sqlDist = 0.0, p2inp = 0.0;
                string tavel_dir = string.Empty;
                SqlGeography shape = null, point = null, circle = null, midpoint = null;
                SqlDataReader read = null;
                Node NearNode1 = null, NearNode2 = null, midNode1 = null, midNode2 = null;
                double initBuffer = 200.0, maxBuffer = 16093.44, buffer = 0.0;
                if (io.MaxNearNodeDistance > 0.0)
                {
                    initBuffer = io.MaxNearNodeDistance;
                    maxBuffer = io.MaxNearNodeDistance;
                }

                point = SqlGeography.STGeomFromText(new SqlChars("POINT(" + io.InpNode.Longitude.ToString("G9") + " " + io.InpNode.Latitude.ToString("G9") + ")"), 4326);

                for (buffer = initBuffer; (io.NearestNodes.Count < MIN_NearestLevelCount) && (buffer <= maxBuffer); buffer = buffer * 2)
                {
                    cmd = io.Con.CreateCommand();
                    cmd.CommandTimeout = 30;
                    cmd.CommandText = "select * from (SELECT s.link_id, s.speed_cat, s.fromlong, s.fromlat, s.tolong, s.tolat, s.shape.STDistance(@point) as dist, " +
                        "s.shape, s.dir_travel, s.fr_spd_lim, s.to_spd_lim, s.len, s.private" + Environment.NewLine +
                        "FROM " + io.GraphName + "_Streets s where (s.ar_auto = 'Y') and (s.speed_cat <> ' ') and (s.ferry_type = 'H')) a" + Environment.NewLine +
                        "where (a.dist <= @buffer) order by a.dist asc";
                    cmd.Parameters.AddWithValue("@buffer", buffer);
                    cmd.Parameters.AddWithValue("@point", point);
                    cmd.Parameters["@point"].UdtTypeName = "geography";
                    read = cmd.ExecuteReader();

                    while (read.Read())
                    {
                        shape = (SqlGeography)(read[7]);
                        len = Convert.ToDouble(read.GetDecimal(11));
                        count = shape.STNumPoints().Value;
                        l1 = 0.0; l2 = 0.0; p1inp = 0.0; travel = 0.0; p1p2 = 0.0;
                        //te1 = 0.0; te2 = 0.0;
                        dist = double.MaxValue; total = 0.0; opTotal = 0.0; circleBufferIndex = 0;
                        sqlDist = Convert.ToDouble(read[6]);

                        while ((circle == null) || (circle.IsNull) || (circle.STIsEmpty()).Value)
                        {
                            circle = point.STBuffer((circleBufferIndex++) * 0.002 + 1.1 * sqlDist);
                            circle = shape.STIntersection(circle);
                        }
                        midpoint = circle.STStartPoint();
                        midNode1 = new Node(new GraphNode(midpoint.Long.Value, midpoint.Lat.Value));
                        midNode2 = new Node(new GraphNode(midpoint.Long.Value, midpoint.Lat.Value));
                        circle = null;

                        // I have no idea what I've done to calculate walking
                        // distance before so I'll just comment out the old one
                        // and re-impelement a new code

                        for (int i = 2; i <= count; i++)
                        {
                            te1 = (io.InpNode.Latitude - shape.STPointN(i - 1).Lat.Value) * (shape.STPointN(i).Lat.Value - shape.STPointN(i - 1).Lat.Value) +
                                (io.InpNode.Longitude - shape.STPointN(i - 1).Long.Value) * (shape.STPointN(i).Long.Value - shape.STPointN(i - 1).Long.Value);
                            te2 = (io.InpNode.Latitude - shape.STPointN(i).Lat.Value) * (shape.STPointN(i - 1).Lat.Value - shape.STPointN(i).Lat.Value) +
                                (io.InpNode.Longitude - shape.STPointN(i).Long.Value) * (shape.STPointN(i - 1).Long.Value - shape.STPointN(i).Long.Value);
                            p1p2 = GreatCircleDistance.LinearDistance2(shape.STPointN(i - 1).Lat.Value, shape.STPointN(i - 1).Long.Value, shape.STPointN(i).Lat.Value, shape.STPointN(i).Long.Value, LinearUnitTypes.Miles);

                            if (te1 * te2 >= 0.0)
                            {
                                p1inp = GreatCircleDistance.LinearDistance2(shape.STPointN(i - 1).Lat.Value, shape.STPointN(i - 1).Long.Value, io.InpNode.Latitude, io.InpNode.Longitude, LinearUnitTypes.Miles);
                                p2inp = GreatCircleDistance.LinearDistance2(shape.STPointN(i).Lat.Value, shape.STPointN(i).Long.Value, io.InpNode.Latitude, io.InpNode.Longitude, LinearUnitTypes.Miles);
                                alpha = (1 + (((p1inp * p1inp) - (p2inp * p2inp)) / (p1p2 * p1p2))) / 2.0;

                                l1 = alpha * p1p2;
                                l2 = Math.Sqrt((p1inp * p1inp) - (l1 * l1));
                                if ((alpha >= 0) && (alpha <= 1) && (l2 < dist))
                                {
                                    dist = l2;
                                    total = l1 + travel;
                                    opTotal = len - travel - l1;
                                }
                            }
                            travel += p1p2;
                        }

                        if (total == 0.0)
                        {
                            p1inp = GreatCircleDistance.LinearDistance2(shape.STPointN(1).Lat.Value, shape.STPointN(1).Long.Value, io.InpNode.Latitude, io.InpNode.Longitude, LinearUnitTypes.Miles);
                            p2inp = GreatCircleDistance.LinearDistance2(shape.STPointN(count).Lat.Value, shape.STPointN(count).Long.Value, io.InpNode.Latitude, io.InpNode.Longitude, LinearUnitTypes.Miles);
                            if (p1inp <= p2inp)
                            {
                                opTotal = len;
                                dist = p1inp;
                            }
                            else
                            {
                                total = len;
                                dist = p2inp;
                            }
                        }

                        nearNodeSpeed = GraphUtils.GetSpeedFromSpeedCat(read.GetString(1));
                        tavel_dir = read.GetString(8);
                        ft_speed = read.GetInt32(9);
                        tf_speed = read.GetInt32(10);

                        if ((ft_speed > 75.0) && (ft_speed <= 100.0)) ft_speed = 75.0;
                        if ((tf_speed > 75.0) && (tf_speed <= 100.0)) tf_speed = 75.0;
                        if ((ft_speed <= 0.0) || (ft_speed > 100.0)) ft_speed = nearNodeSpeed;
                        if ((tf_speed <= 0.0) || (tf_speed > 100.0)) tf_speed = nearNodeSpeed;

                        if (!io.NearestNodes.ContainsKey(dist)) io.NearestNodes.Add(dist, new List<Node>());
                        NearNode1 = new Node(new GraphNode(Convert.ToDouble(read.GetDecimal(2)), Convert.ToDouble(read.GetDecimal(3))),
                            total / ft_speed, total, null);
                        NearNode2 = new Node(new GraphNode(Convert.ToDouble(read.GetDecimal(4)), Convert.ToDouble(read.GetDecimal(5))),
                            opTotal / tf_speed, opTotal, null);

                        midNode1.PreviousNode = NearNode2;
                        midNode2.PreviousNode = NearNode1;
                        NearNode1.PreviousNode = midNode1;
                        NearNode2.PreviousNode = midNode2;

                        switch (tavel_dir)
                        {
                            case "B":
                                io.NearestNodes[dist].Add(NearNode1);
                                io.NearestNodes[dist].Add(NearNode2);
                                break;
                            case "F":
                                if (io.IsSource)
                                {
                                    io.NearestNodes[dist].Add(NearNode2);
                                }
                                else
                                {
                                    io.NearestNodes[dist].Add(NearNode1);
                                }
                                break;
                            case "T":
                                if (io.IsSource)
                                {
                                    io.NearestNodes[dist].Add(NearNode1);
                                }
                                else
                                {
                                    io.NearestNodes[dist].Add(NearNode2);
                                }
                                break;
                            default:
                                throw new NotSupportedException("Direction not supported.");
                        } // switch direction of travel

                        #region Old Version

                        /* 
						tempNode = new Node(Convert.ToDouble(read.GetDecimal(2)), Convert.ToDouble(read.GetDecimal(3)));
						tempCost = GreatCircleDistance.LinearDistance2(inpNode.Latitude, inpNode.Longitude, tempNode.Latitude, tempNode.Longitude, LinearUnitTypes.Miles);
						if ((tempCost < nearNodeCost) || (nearNodeCost < 0.0))
						{
							nearNodeCost = tempCost;
							nearNode = tempNode;
							nearNodeSpeed = GraphUtils.GetSpeedFromSpeedCat(read.GetString(1));
						}
						tempNode = new Node(Convert.ToDouble(read.GetDecimal(4)), Convert.ToDouble(read.GetDecimal(5)));
						tempCost = GreatCircleDistance.LinearDistance2(inpNode.Latitude, inpNode.Longitude, tempNode.Latitude, tempNode.Longitude, LinearUnitTypes.Miles);
						if ((tempCost < nearNodeCost) || (nearNodeCost < 0.0))
						{
							nearNodeCost = tempCost;
							nearNode = tempNode;
							nearNodeSpeed = GraphUtils.GetSpeedFromSpeedCat(read.GetString(1));
						}						
					*/

                        #endregion

                    } // while read
                    read.Close();

                } // while more streets to read
                io.DiscoveredRadius = buffer;
            }
            catch (System.Threading.ThreadAbortException te)
            {
                throw te;
            }
            return io;
        }

        // This method searches the DB for nearest node to to input node and then it adds
        // the nearest intersection node as a neighbor to this virtual node and returns it (non-spatial search)
        /*
		public static double[] FindNearestNode(string graphName, ref Node inpNode, SqlConnection con)
		{
			if (con.State != ConnectionState.Open) con.Open();

			var cmd = con.CreateCommand();
			double nearNodeSpeed = 0.0;
			Node nearNode = null, tempNode = null;
			double nearNodeCost = Node.Cost_Undeterm, tempCost = 0.0;
			cmd.CommandTimeout = 0;
			SqlDataReader read = null;

			double bbox = 0.0001;
			const int ratio = 20;
			bool found = false;

			while ((!found) && (bbox < 0.1f))
			{
				cmd.CommandText = "select [link_id], [speed_cat], [fromlong], [fromlat], [tolong], [tolat] from " + graphName + "_Streets " +
					"where ((abs(fromlat - " + inpNode.Latitude + ") < " + bbox + ") and (abs(fromlong - (" + inpNode.Longitude + ")) < " + bbox + ")) " +
					"or ((abs(tolat - " + inpNode.Latitude + ") < " + bbox + ") and (abs(tolong - (" + inpNode.Longitude + ")) < " + bbox + "))";
				read = cmd.ExecuteReader();

				while (read.Read())
				{
					tempNode = new Node(new GraphNode(Convert.ToDouble(read.GetDecimal(2)), Convert.ToDouble(read.GetDecimal(3))));
					tempCost = GreatCircleDistance.LinearDistance2(inpNode.Latitude, inpNode.Longitude, tempNode.Latitude, tempNode.Longitude, LinearUnitTypes.Miles);
					if ((tempCost < nearNodeCost) || (nearNodeCost < 0.0))
					{
						nearNodeCost = tempCost;
						nearNode = tempNode;
						nearNodeSpeed = GraphUtils.GetSpeedFromSpeedCat(read.GetString(1));
						found = true;
					}
					tempNode = new Node(new GraphNode(Convert.ToDouble(read.GetDecimal(4)), Convert.ToDouble(read.GetDecimal(5))));
					tempCost = GreatCircleDistance.LinearDistance2(inpNode.Latitude, inpNode.Longitude, tempNode.Latitude, tempNode.Longitude, LinearUnitTypes.Miles);
					if (tempCost < nearNodeCost)
					{
						nearNodeCost = tempCost;
						nearNode = tempNode;
						nearNodeSpeed = GraphUtils.GetSpeedFromSpeedCat(read.GetString(1));
						found = true;
					}
				}
				read.Close();
				bbox *= ratio;
			} // while not found			

			if ((nearNode != null) && (nearNodeCost < 5))
			{
				inpNode = nearNode;
				return new double[2] { nearNodeCost, nearNodeCost / nearNodeSpeed };
			}
			else
			{
				inpNode = new Node();
				return new double[2] { Node.Cost_NoNearNode, Node.Cost_NoNearNode };
			}
		}
        */
    }
}