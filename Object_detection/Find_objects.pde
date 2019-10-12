int labelPixelsReturnL(int[] labels, int[] colours, int x, int l) {
  if (colours[x] == fgColour) {
    int minLabel = smallestKernelLabel(labels, w, x);
    if (minLabel == 0) {
      // neighbours A, B, C and D are unlabelled (equal to zero)
      labels[x] = l;
      l += 1;
    } else {
      labels[x] = minLabel;
      setKernelLabelsIfForeground(labels, kinect.rgbImage().pixels, x, w, minLabel);
    }
  }
  return l;
}

int[] kernelLabels(int[] labels, int w, int x) {
  int[] neighbours = kernelIndices(x, w);
  int[] neighbourLabels = new int[neighbours.length];

  for (int i = 0; i < neighbours.length; i += 1) {
    // Check each element of the neightbours A, B, C and D and
    // if the label is bigger than the maxLabel, increase maxLabel
    if (neighbours[i] >= 0) {
      neighbourLabels[i] = labels[neighbours[i]];
    }
  }
  return neighbourLabels;
}
int smallestKernelLabel(int[] labels, int w, int x) {
  int minLabel = Integer.MAX_VALUE;
  int[] neighbourLabels = kernelLabels(labels, w, x);
  for (int i = 0; i < neighbourLabels.length; i += 1) {
    int neighbourLabel = neighbourLabels[i];
    if (neighbourLabel > 0) {
      minLabel = min(minLabel, neighbourLabel);
    }
  }
  if (minLabel == Integer.MAX_VALUE) {
    // All neighbours have 0 label
    return 0;
  } else {
    return minLabel;
  }
}

void setKernelLabelsIfForeground(int[] labels, color[] colours, int x, int w, int l) {
  int[] neighbours = kernelIndices(x, w);
  for (int i = 0; i < neighbours.length; i += 1) {
    int neighbourIndex = neighbours[i];
    if (neighbourIndex >= 0) {
      if (colours[neighbourIndex] == fgColour) {
        labels[neighbourIndex] = l;
      }
    }
  }
}

int[] kernelIndices(int i, int w) {
  int[] xOffsets = {
    -1, 0, 1, -1
  };
  int[] yOffsets = {
    -1, -1, -1, 0
  };
  int x = xFromIndex(i, w);
  int y = yFromIndex(i, w);
  int[] neighbours = new int[4];
  for (int j = 0; j < 4; j += 1) {
    neighbours[j] = indexFromXY(x + xOffsets[j], y + yOffsets[j], w);
  }
  return neighbours;
}

int xFromIndex(int i, int w) {
  return i % w;
}

int yFromIndex(int i, int w) {
  return i / w;
}

int indexFromXY(int x, int y, int w) {
  if (x >= 0 && y >= 0 && x < w) {
    return y * w + x;
  } else {
    return -1;
  }
}
