#include <iostream>
#include <vector>
#include <queue>
using namespace std;

class Solution {
    using P = pair<int, int> ;
    vector<vector<P>> adjList;
    const int MOD = 1e9+7;

    int minTimeToReachDest_DFS(int src, int dst, vector<bool>& visited) {
        if(src == dst)
            return 0;

        visited[src] = true;
        int minimumTime = INT_MAX;

        for(auto& [neighbor, edgeWeight] : adjList[src]) {
            if(!visited[neighbor]) {
                int nextTime = minTimeToReachDest_DFS(neighbor, dst, visited);
                if(nextTime != INT_MAX) {
                    minimumTime  = min(minimumTime, nextTime + edgeWeight);
                }
            }
        }

        visited[src] = false;
        return minimumTime;
    } 
    // Gives TLE at 4

    int minTimeToReachDest_Dijsktra(int n, int src, int dst) {
        vector<int> minimumTime(n, INT_MAX);
        minimumTime[src] = 0;

        priority_queue<P, vector<P>, greater<P>> minHeap;
        minHeap.push({0, src});

        while(!minHeap.empty()) {
            auto [currTime, node] = minHeap.top(); minHeap.pop();

            if(node == dst)
                return currTime;
            
            for(auto& [neighbor, edgeWeight] : adjList[node]) {
                int newTime = currTime + edgeWeight;
                if(minimumTime[neighbor] > newTime) {
                    minimumTime[neighbor] = newTime;
                    minHeap.push({newTime, neighbor});
                }
            }
        }

        return -1;
    } 
    // Gives TLE at 38

    int countPathsToDestInTime(vector<vector<int>>& dp, int time, int src, int dst) {
        if(src == dst)
            return (time == 0);

        if(time < 0)
            return 0;

        if(dp[src][time] != -1)
            return dp[src][time];

        int countPaths = 0;

        for(auto& [neighbor, edgeWeight] : adjList[src]) 
            countPaths = (countPaths + countPathsToDestInTime(dp, time - edgeWeight, neighbor, dst)) % MOD;

        return dp[src][time] = countPaths;
    }

public:
    int countPaths(int n, vector<vector<int>>& edges) {
        adjList.resize(n);

        for(auto& edge : edges) { // Construct the graph
            int u = edge[0];
            int v = edge[1];
            int w = edge[2];
            adjList[u].push_back({v, w});
            adjList[v].push_back({u, w});
        }

        int shortestTime = minTimeToReachDest_Dijsktra(n, 0, n-1);

        vector<vector<int>> dp(n, vector<int>(shortestTime + 1, -1));
        return countPathsToDestInTime(dp, shortestTime, 0, n-1);
    }
};

class PrioritizeShortestTime {
    using P = pair<int, int> ;
    vector<vector<P>> adjList;
    const int MOD = 1e9+7;

    int dijkstra(int n, int src, int dst) {
    }

public:
    int countPaths(int n, vector<vector<int>>& edges) {
        adjList.resize(n);

        for(auto& edge : edges) { // Construct the graph
            int u = edge[0];
            int v = edge[1];
            int w = edge[2];
            adjList[u].push_back({v, w});
            adjList[v].push_back({u, w});
        }

        return dijkstra(n, 0, n-1); 
    }
};

// Driver code
int main() {
    ios_base::sync_with_stdio(NULL);
    cin.tie(nullptr);

    return 0;
}
// Link: https://leetcode.com/problems/number-of-ways-to-arrive-at-destination/description/