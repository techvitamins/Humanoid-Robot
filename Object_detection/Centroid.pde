void draw_centroids(int[] labels, int w, ArrayList labelsToShow) {
  int[][] xy = centroids(labels, w, labelsToShow); // Find all the centroids
  for (int l = 0; l < xy.length; l +=1) {
    draw_centroid(xy[l][0], xy[l][1]);
  }
}

int[][] centroids(int[] labels, int w, ArrayList labelsToShow) {
  int[][] ij = new int[3][2];
  // Moment 00
  ij[0][0] = 0;
  ij[0][1] = 0;
  // Moment 10
  ij[1][0] = 1;
  ij[1][1] = 0;
  // Moment 01
  ij[2][0] = 0;
  ij[2][1] = 1;

  int[][] m = moments(labels, w, labelsToShow, ij); 
  int[][] xy = new int[labelsToShow.size()][2];
  for (int label = 0; label < m.length; label += 1) {
    int m00 = m[label][0];
    int m10 = m[label][1];
    int m01 = m[label][2];
    // Centroid x
    if (m00 > 0) {
      xy[label][0] = m10 / m00;
      // Centroid y
      xy[label][1] = m01 / m00;
    } else {
      xy[label][0] = -1;
      xy[label][1] = -1;
    }
  }
  return xy;
}

// Get a set of i-jth moments of all groups in labelsToShow from an array of labels
// with width w. ij is an array of 2-element integer arrays as follows:
// [[i0, j0], [i1, j1], ...]
// For each label, returns an array of moments
int[][] moments(int[] labels, int w, ArrayList<Integer> labelsToShow, int[][] ij) {
  int[][] m = new int[labelsToShow.size()][ij.length];
  for (int k = 0; k < labels.length; k += 1) {
    int x = xFromIndex(k, w);
    int y = yFromIndex(k, w);
    if (labels[k] > 0) {
      for (int n = 0; n < labelsToShow.size (); n += 1) {
        int label = labelsToShow.get(n);
        if (labels[k] == label) {
          for (int moment = 0; moment < ij.length; moment += 1) {
            int i = ij[moment][0];
            int j = ij[moment][1];
            m[n][moment] += Math.pow(x, i) * Math.pow(y, j); //<>//
          }
        }
      }
    }
  }
  return m;
}
