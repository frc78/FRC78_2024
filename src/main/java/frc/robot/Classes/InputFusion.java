// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Classes;

public class InputFusion {
  public static double weightedFusion(double[] inputs, double[] weights) {
    // Calculate the total weight
    double totalWeight = 0;
    for (double weight : weights) {
      totalWeight += weight;
    }

    // Normalize the weights
    double[] normalizedWeights = weights;
    for (int i = 0; i < weights.length; i++) {
      normalizedWeights[i] = weights[i] / totalWeight;
    }

    // Calculate the weighted sum
    double output = 0;
    for (int i = 0; i < inputs.length; i++) {
      output += inputs[i] * normalizedWeights[i];
    }

    // Return the output
    return output;
  }
}
