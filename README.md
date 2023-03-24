# Informative-Path-Planning23

The Informative Path Planning (IPP) algorithm was 



https://youtu.be/Fi9cuvjKcn8



The IPP algorithm is shown in a YouTube video\footnote{\url{https://youtu.be/Fi9cuvjKcn8}} exploring the map from which the simulation snapshots in FIG.~\ref{fig:simulation_snapshot} were obtained, as shown in FIG.~\ref{fig:IPP-matlab-simulation}. The video has a playback speed that is four times faster than the original simulation. It is noteworthy that the IPP algorithm was able to explore the map in 4.59 seconds without the visual aid presented in the video. With the visualization, the IPP algorithm takes more than 20 minutes to explore the map.
 


The visualization contains six figures in total, five of which are updated continuously. A description of each of the figures is described as follows:


\begin{itemize}
    \item  Ground-Truth Map (top-left figure): shows the map that the mobile robot needs to explore, which it does not know anything about.
    \item  Egocentric Map (top-middle figure): shows the map that the mobile robot updates as it explores the unknown map.
    \item  Expansion Map (top-right figure): shows the square sensor model used in the mobiel robot.
    \item  Global Planner Path to Goal (bottom-left figure): shows how the A-Star algorithm generates a path to an unexplored region after the local planner is deactivated and there still exists regions to explore.
    \item  Expansion Model Update 5X5 (bottom-middle figure): Mobile Robot's zoomed in point of view of the expansion map.
    \item  All Time Poses of Robot (bottom-right figure): shows the path that the mobile robot takes to explore the unknown map, and it is refreshed every time the IPP algorithm switches between the global and local planner.
\end{itemize}
