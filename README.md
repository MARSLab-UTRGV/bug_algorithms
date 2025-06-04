# bug-algorithms
Bug Algorithms in Webots

Consist of Bug Algorithms:
1. Bug 0+ (Common Sense)
2. Bug 2+
3. Rev 1
4. Rev 2
5. Alg 1
6. Alg 2
7. ComboBug
8. RevComboBug

These bug algorithms are all run on e-puck robot simulations in Webots. They can be adjusted to fit your Webot build and system, however the logic stands to be the same. 
Bug 0+ and Bug 2+ are updated systems of their original counter parts. They have logic added to ensure they are able to reach the goal without getting stuck within an obstacle. This can be taken out of the code to create analysis with the original base algorithm.

Algorithms 1-7 have been evaluated and compared. For the time being please reach out to me for the results of said comparative analysis. The 8th algorithm is a revision of the 7th to improve upon and further merge differences in the algorithms to create a more effective localized path planning. This algorithm is in the base stages of revision and debugging as of 6/4/2025.

As of 6/4/2025 there is a new set of obstacle maps being created to involve more complex situations. The preliminary ones are fairly simple and were meant to test the boundaries of the new ComboBug algorithm. The RevComboBug Algorithm is expected to perform better than ComboBug in some situations, but a more complex environment is needed to set them further apart and prove the effectiveness of the new algorithm 8.

If you are unfamiliar with webots, I suggest these videos to help you understand the world building process. https://youtube.com/playlist?list=PLbEU0vp_OQkUwANRMUOM00SXybYQ4TXNF&si=ejOqIpJbeA-LQ5_N
