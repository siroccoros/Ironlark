// A finding from building this presentation: today's empty-world recordings fail check-recording at their tail.
import flight from "../generated/flight-data.js";

export default function tail(root) {
  const runs = flight.failedRecordings;
  root.innerHTML = `<p class="k">Found while building this · check-recording on today's main</p>
    <h5>Two of today's three recordings fail <code>check-recording</code></h5>
    <p>Both empty-world flights passed <code>./ironlark check --record</code>, and in both the ROS bridge acknowledged every message the
      controller sent. Their bags still end early on some topics:</p>
    <table><tr><th>run</th><th>acknowledged by the bridge</th><th>check-recording</th></tr>
      ${runs.map((r) => `<tr><td>${r.run}</td><td>${r.acknowledged.toLocaleString("en")} of ${r.acknowledged.toLocaleString("en")}</td><td>${r.problems.join("<br>")}</td></tr>`).join("")}
    </table>
    <p>The stream's own accounting ends at the bridge: <code>end_sensing()</code> (<code>ironlark:303</code>) waits until the bridge has
      read and published every message, and the <code>finally</code> block then sends SIGINT to the recorder. Messages published but not
      yet written by rosbag2 at that moment would be lost, which fits losses that appear only at the end, on different topics
      each time. That cause is a hypothesis, not yet tested. The forest recording made between the two passed.</p>`;
}
