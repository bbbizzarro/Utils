using System.Collections;
using System.Collections.Generic;
using System;
using Utils;
using Godot;

namespace Utils {

public interface IPathGrid {
	List<Vector2Int> GetOpenNeighbors(int x, int y);
	bool IsOpen(int x, int y);
}

public class PathFinder {

	int maxSearchSize;
	IPathGrid grid;
	BinaryHeap<Vector2Int> pq;
	Dictionary<Vector2Int, TableEntry> table;
	Vector2Int start;
	Vector2Int end;

	public PathFinder(int maxSearchSize, IPathGrid grid) {
		this.grid = grid;
		this.maxSearchSize = maxSearchSize;
		table = new Dictionary<Vector2Int, TableEntry>();
		pq = new BinaryHeap<Vector2Int>(maxSearchSize, new NodeComparer(table));
	}

	#region Methods
	// Iterator ============
	public void StartFindPath(Vector2Int start, Vector2Int end) {
		pq.Clear();
		table.Clear();
		Add(start, start, 0, (end - start).Magnitude());
		this.start = start;
		this.end = end;
	}

	public Vector2Int FindNext() {
		if (!pq.IsEmpty()) {
			Vector2Int next = pq.Pop();
			foreach (var n in grid.GetOpenNeighbors(next.x, next.y)) {
				float costFromStart = table[next].dist + 
									  next.Manhattan(n) ;
				float H = (end - n).Magnitude();
				//float H = end.Euclidian(n);
				Add(n, next, 0, H);
				if (n == end) {
					pq.Clear();
					return next;
				}
			}
			return next;
		}
		return Vector2Int.Zero;
	}
	// =====================

	public void Intercept() {
	}

	/*
	A heuristic is admissible if it never overestimates the true cost to a nearest goal.

	A heuristic is consistent if, when going from neighboring nodes a to b, the heuristic difference/step cost
	never overestimates the actual step cost
	*/

	public List<Vector2Int> FindPath(Vector2Int start, Vector2Int end) {
		if (!grid.IsOpen(end.x, end.y)) return new List<Vector2Int>();
		RandomNumberGenerator rng = new RandomNumberGenerator();
		rng.Randomize();
		StartFindPath(start, end);
		Vector2Int next = start;
		int iters = 0;
		while (!pq.IsEmpty() && iters < maxSearchSize) {
			next = pq.Pop();
			foreach (var n in grid.GetOpenNeighbors(next.x, next.y)) {
				float costFromStart = table[next].dist + 
									  next.Manhattan(n) ;

				// HEURISTIC ------------------
				float H = (end - n).Magnitude();
				//float H = end.Euclidian(n);
				// ----------------------------

				float randFactor = 0f;
				Add(n, next, randFactor * rng.Randf(), H);
				if (n == end) {
					return Trace(n);
				}
			}
			iters += 1;
		}
		return new List<Vector2Int>();
	}

	public List<Vector2> FindClosest(Vector2 startWorldPos) {
		return null;
	}
	#endregion


	#region Utils
	private bool IsStart(Vector2Int node) {
		return node == table[node].prev;
	}

	private List<Vector2Int> Trace(Vector2Int end) {
		Stack<Vector2Int> revPath = new Stack<Vector2Int>();
		Vector2Int next = end;
		while (!IsStart(next)) {
			revPath.Push(next);
			next = table[next].prev;
		}
		revPath.Push(start);
		return new List<Vector2Int>(revPath);
	}

	public float GetDistance(Vector2Int p) {
		if (table.ContainsKey(p)) {
			return table[p].dist;
		}
		else {
			return -1;
		}
	}

	public float GetHeuristic(Vector2Int p) {
		if (table.ContainsKey(p)) {
			return table[p].H;
		}
		else {
			return -1;
		}
	}

	private void Add(Vector2Int node, Vector2Int prev, float dist, float H) {
		float weight = dist + H;
		if (table.ContainsKey(node)) {
			if (table[node].Weight() > weight) {
				table[node].dist = dist;
				table[node].H = H;
				pq.Update(node);
			}
			return;
		}
		else { 
			table.Add(node, new TableEntry(prev, dist, H));
			pq.Insert(node);
		}
	}
	#endregion

	#region Classes
	public class TableEntry {
		public Vector2Int prev;
		public float dist;
		public float H;

		public TableEntry(Vector2Int prev, float dist, float H) {
			this.prev = prev;
			this.dist = dist;
			this.H = H;
		}

		public float Weight() {return dist + H; }

		public override string ToString() {
			return String.Format("Prev: {0}, Dist: {1}, H: {2}", prev, dist, H);
		}
	}

	public class NodeComparer : IComparer<Vector2Int> {
		Dictionary<Vector2Int, TableEntry> table;
		public NodeComparer(Dictionary<Vector2Int, TableEntry> table) {
			this.table = table;
		}
		public int Compare(Vector2Int x, Vector2Int y) {
			if (!table.ContainsKey(x) || !table.ContainsKey(y) ||
				table[x].Weight() == table[y].Weight()) {
				return 0;
			}
			if (table[x].Weight() > table[y].Weight()) {
				return 1;
			}
			return -1;
		}
	}
	#endregion
}

}