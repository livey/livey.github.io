import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

def plot_mixture_distributions():
    """
    Original plot: Y1 = X + Z1, Y2 = X + Z2
    """
    # Parameters
    mu_x = 0.0
    sigma_x = 0.4
    
    # Distribution 1 parameters
    z1_min = -0.3
    z1_max = 0.0
    
    # Distribution 2 parameters
    z2_min = 0.0
    z2_max = 0.3
    
    y_grid = np.linspace(-1.5, 1.5, 1000)
    
    # PDF Y1 (Z1 ~ U[-0.3, 0])
    cdf_upper_1 = norm.cdf((y_grid - mu_x - z1_min) / sigma_x)
    cdf_lower_1 = norm.cdf((y_grid - mu_x - z1_max) / sigma_x)
    pdf_y1_analytical = (cdf_upper_1 - cdf_lower_1) / (z1_max - z1_min)
    
    # PDF Y2 (Z2 ~ U[0, 0.3])
    cdf_upper_2 = norm.cdf((y_grid - mu_x - z2_min) / sigma_x)
    cdf_lower_2 = norm.cdf((y_grid - mu_x - z2_max) / sigma_x)
    pdf_y2_analytical = (cdf_upper_2 - cdf_lower_2) / (z2_max - z2_min)
    
    # PDF of X
    pdf_x = norm.pdf(y_grid, mu_x, sigma_x)
    
    # Scaled PDFs
    scaled_pdf_y1 = 0.25 * pdf_y1_analytical
    scaled_pdf_y2 = 0.25 * pdf_y2_analytical
    scaled_pdf_x = 0.5 * pdf_x
    
    # Plotting
    plt.figure(figsize=(10, 6))
    
    plt.fill_between(y_grid, scaled_pdf_y1, color='dodgerblue', alpha=0.5, label='Right Turn')
    plt.fill_between(y_grid, scaled_pdf_y2, color='tomato', alpha=0.5, label='Left Turn')
    plt.fill_between(y_grid, scaled_pdf_x, color='limegreen', alpha=0.4, label='Straight')
    
    plt.plot(y_grid, scaled_pdf_y1, color='navy', linestyle='-', linewidth=2)
    plt.plot(y_grid, scaled_pdf_y2, color='darkred', linestyle='-', linewidth=2)
    plt.plot(y_grid, scaled_pdf_x, color='darkgreen', linestyle='--', linewidth=2)
    
    plt.title(f'Density of Different Modes (Mixture)')
    plt.xlabel('X')
    plt.ylabel('Probability Density')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    output_file = 'distribution_plot_mixture.png'
    plt.savefig(output_file, dpi=500)
    print(f"Mixture plot saved to {output_file}")


def plot_split_distributions():
    """
    New plot: 
    X ~ N(0, 0.4^2)
    Y1 = X_pos + Z1, where X_pos = X | X > 0, Z1 ~ U[0, 0.3]
    Y2 = X_neg + Z2, where X_neg = X | X < 0, Z2 ~ U[-0.3, 0]
    """
    # Parameters
    mu_x = 0.0
    sigma_x = 0.4
    z_range = 0.3 # Length of uniform interval
    
    # Grid
    x = np.linspace(-1.5, 1.5, 1000)
    
    # PDF of X (Complete Gaussian)
    pdf_x = norm.pdf(x, mu_x, sigma_x)
    
    # --- Y1 Distribution (Positive X shifted right) ---
    # Convolution of Truncated Normal (X>0) and Uniform(0, 0.3)
    # The PDF of X in the positive region is f_X(x).
    # Since we are considering the component of the mixture X corresponding to X>0,
    # the total mass is 0.5. The 'conditional' PDF would be 2*f_X(x).
    # IF we want to plot the distribution of Y1 as if it replaces the positive part of X:
    # We essentially take the mass f_X(x) for x>0 and smear it by Z1.
    # So we don't multiply by 2 if we want the relative density to original X mass.
    # Wait, the user asked for "distribution of X, Y1, Y2".
    # Usually implies comparing densities.
    # Let's assume we want to show how the positive mass of X moves to Y1,
    # and negative mass moves to Y2.
    # Mass of X_pos is 0.5. Y1 is that mass convolved. So integral of Y1 should be 0.5.
    
    # Formula for convolution of f(x) * I(x>0) and Uniform(0, a):
    # g(y) = (1/a) * Integral_{y-a}^{y} f(t) dt   (where f(t)=0 if t<0)
    #      = (1/a) * [F_X(y) - F_X(max(0, y-a))]
    # Note: F_X here is the integral of the original Gaussian PDF.
    
    # Y1: Shift by Z1 in [0, 0.3]. a = 0.3.
    # Support of Y1 starts at 0.
    
    term1_y1 = norm.cdf(x, mu_x, sigma_x)
    term2_y1 = norm.cdf(np.maximum(0, x - z_range), mu_x, sigma_x)
    pdf_y1 = (1.0 / z_range) * (term1_y1 - term2_y1)
    pdf_y1[x <= 0] = 0
    
    # --- Y2 Distribution (Negative X shifted left) ---
    # Shift by Z2 in [-0.3, 0].
    # Effectively convolving f(x)*I(x<0) with Uniform(-0.3, 0).
    # Let Z' = -Z2 ~ U[0, 0.3]. Y2 = X_neg - Z'.
    # Or just use raw formula:
    # g(y) = (1/a) * Integral_{y-b}^{y-a} f(t) dt
    # a = -0.3, b = 0.
    # Integral from y to y+0.3 of f(t)*I(t<0).
    # Interval is [y, y+0.3]. Intersection with (-inf, 0) is [y, min(0, y+0.3)].
    
    upper_limit_y2 = np.minimum(0, x + z_range)
    # Low limit is y (since we integrate f(t) from y-0 to y-(-0.3) -> [y, y+0.3])
    # But strictly, convolution of f and U[a,b] is 1/(b-a) * int(y-b to y-a) f(t) dt.
    # Z2 ~ [-0.3, 0]. a=-0.3, b=0.
    # Integral from y-0 to y-(-0.3) => [y, y+0.3].
    
    term1_y2 = norm.cdf(upper_limit_y2, mu_x, sigma_x)
    term2_y2 = norm.cdf(x, mu_x, sigma_x)
    # The integral should be positive, so F(upper) - F(lower).
    # If y > 0, then interval [y, y+0.3] is all > 0, so integral of X_neg is 0.
    # If y < -0.3, interval [y, y+0.3] is all < 0.
    
    pdf_y2 = (1.0 / z_range) * (term1_y2 - term2_y2)
    # Fix potential negative values due to numerical dust or logic
    pdf_y2[pdf_y2 < 0] = 0 
    pdf_y2[x >= 0] = 0
    
    # Plotting
    # Scaled PDFs
    scaled_pdf_x = 0.5 * pdf_x
    scaled_pdf_y1 = 0.5 * pdf_y1
    scaled_pdf_y2 = 0.5 * pdf_y2

    # Plotting
    plt.figure(figsize=(10, 6))
    
    # X - Straight (Full Gaussian)
    plt.fill_between(x, scaled_pdf_x, color='limegreen', alpha=0.3, label='Straight')
    plt.plot(x, scaled_pdf_x, color='darkgreen', linestyle='--', linewidth=2)
    
    # Y1 - Positive Part Shifted
    plt.fill_between(x, scaled_pdf_y1, color='dodgerblue', alpha=0.5, label='Right Turn')
    plt.plot(x, scaled_pdf_y1, color='navy', linestyle='-', linewidth=2)
    
    # Y2 - Negative Part Shifted
    plt.fill_between(x, scaled_pdf_y2, color='tomato', alpha=0.5, label='Left Turn')
    plt.plot(x, scaled_pdf_y2, color='darkred', linestyle='-', linewidth=2)
    
    plt.title(f'Distributions of Different Modes (Mixture)')
    plt.xlabel('X')
    plt.ylabel('Probability Density')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    output_file = 'distribution_plot_split.png'
    plt.savefig(output_file, dpi=500)
    print(f"Split plot saved to {output_file}")


def plot_mixture_distributions_2():
    """
    Original plot: Y1 = X + Z1, Y2 = X + Z2
    """
    # Parameters
    mu_x = 0.0
    sigma_x = 0.05
    
    # Distribution 1 parameters
    z1_min = -0.3
    z1_max = 0.0
    
    # Distribution 2 parameters
    z2_min = 0.0
    z2_max = 0.3
    
    y_grid = np.linspace(-1.5, 1.5, 1000)
    
    # PDF Y1 (Z1 ~ U[-0.3, 0])
    cdf_upper_1 = norm.cdf((y_grid - mu_x - z1_min) / sigma_x)
    cdf_lower_1 = norm.cdf((y_grid - mu_x - z1_max) / sigma_x)
    pdf_y1_analytical = (cdf_upper_1 - cdf_lower_1) / (z1_max - z1_min)
    
    # PDF Y2 (Z2 ~ U[0, 0.3])
    cdf_upper_2 = norm.cdf((y_grid - mu_x - z2_min) / sigma_x)
    cdf_lower_2 = norm.cdf((y_grid - mu_x - z2_max) / sigma_x)
    pdf_y2_analytical = (cdf_upper_2 - cdf_lower_2) / (z2_max - z2_min)
    
    # PDF of X
    pdf_x = norm.pdf(y_grid, mu_x, sigma_x)
    
    # Scaled PDFs
    scaled_pdf_y1 = 0.25 * pdf_y1_analytical
    scaled_pdf_y2 = 0.25 * pdf_y2_analytical
    scaled_pdf_x = 0.5 * pdf_x
    
    # Plotting
    plt.figure(figsize=(10, 6))
    
    plt.fill_between(y_grid, scaled_pdf_y1, color='dodgerblue', alpha=0.5, label='Right Turn')
    plt.fill_between(y_grid, scaled_pdf_y2, color='tomato', alpha=0.5, label='Left Turn')
    plt.fill_between(y_grid, scaled_pdf_x, color='limegreen', alpha=0.4, label='Straight')
    
    plt.plot(y_grid, scaled_pdf_y1, color='navy', linestyle='-', linewidth=2)
    plt.plot(y_grid, scaled_pdf_y2, color='darkred', linestyle='-', linewidth=2)
    plt.plot(y_grid, scaled_pdf_x, color='darkgreen', linestyle='--', linewidth=2)
    
    plt.title(f'Density of Different Modes (Mixture)')
    plt.xlabel('X')
    plt.ylabel('Probability Density')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    output_file = 'distribution_plot_mixture-2.png'
    plt.savefig(output_file, dpi=500)
    print(f"Mixture plot saved to {output_file}-2")


def plot_split_distributions_2():
    """
    New plot: 
    X ~ N(0, 0.4^2)
    Y1 = X_pos + Z1, where X_pos = X | X > 0, Z1 ~ U[0, 0.3]
    Y2 = X_neg + Z2, where X_neg = X | X < 0, Z2 ~ U[-0.3, 0]
    """
    # Parameters
    mu_x = 0.0
    sigma_x = 0.05
    z_range = 0.3 # Length of uniform interval
    
    # Grid
    x = np.linspace(-1.5, 1.5, 1000)
    
    # PDF of X (Complete Gaussian)
    pdf_x = norm.pdf(x, mu_x, sigma_x)
    
    # --- Y1 Distribution (Positive X shifted right) ---
    # Convolution of Truncated Normal (X>0) and Uniform(0, 0.3)
    # The PDF of X in the positive region is f_X(x).
    # Since we are considering the component of the mixture X corresponding to X>0,
    # the total mass is 0.5. The 'conditional' PDF would be 2*f_X(x).
    # IF we want to plot the distribution of Y1 as if it replaces the positive part of X:
    # We essentially take the mass f_X(x) for x>0 and smear it by Z1.
    # So we don't multiply by 2 if we want the relative density to original X mass.
    # Wait, the user asked for "distribution of X, Y1, Y2".
    # Usually implies comparing densities.
    # Let's assume we want to show how the positive mass of X moves to Y1,
    # and negative mass moves to Y2.
    # Mass of X_pos is 0.5. Y1 is that mass convolved. So integral of Y1 should be 0.5.
    
    # Formula for convolution of f(x) * I(x>0) and Uniform(0, a):
    # g(y) = (1/a) * Integral_{y-a}^{y} f(t) dt   (where f(t)=0 if t<0)
    #      = (1/a) * [F_X(y) - F_X(max(0, y-a))]
    # Note: F_X here is the integral of the original Gaussian PDF.
    
    # Y1: Shift by Z1 in [0, 0.3]. a = 0.3.
    # Support of Y1 starts at 0.
    
    term1_y1 = norm.cdf(x, mu_x, sigma_x)
    term2_y1 = norm.cdf(np.maximum(0, x - z_range), mu_x, sigma_x)
    pdf_y1 = (1.0 / z_range) * (term1_y1 - term2_y1)
    pdf_y1[x <= 0] = 0
    
    # --- Y2 Distribution (Negative X shifted left) ---
    # Shift by Z2 in [-0.3, 0].
    # Effectively convolving f(x)*I(x<0) with Uniform(-0.3, 0).
    # Let Z' = -Z2 ~ U[0, 0.3]. Y2 = X_neg - Z'.
    # Or just use raw formula:
    # g(y) = (1/a) * Integral_{y-b}^{y-a} f(t) dt
    # a = -0.3, b = 0.
    # Integral from y to y+0.3 of f(t)*I(t<0).
    # Interval is [y, y+0.3]. Intersection with (-inf, 0) is [y, min(0, y+0.3)].
    
    upper_limit_y2 = np.minimum(0, x + z_range)
    # Low limit is y (since we integrate f(t) from y-0 to y-(-0.3) -> [y, y+0.3])
    # But strictly, convolution of f and U[a,b] is 1/(b-a) * int(y-b to y-a) f(t) dt.
    # Z2 ~ [-0.3, 0]. a=-0.3, b=0.
    # Integral from y-0 to y-(-0.3) => [y, y+0.3].
    
    term1_y2 = norm.cdf(upper_limit_y2, mu_x, sigma_x)
    term2_y2 = norm.cdf(x, mu_x, sigma_x)
    # The integral should be positive, so F(upper) - F(lower).
    # If y > 0, then interval [y, y+0.3] is all > 0, so integral of X_neg is 0.
    # If y < -0.3, interval [y, y+0.3] is all < 0.
    
    pdf_y2 = (1.0 / z_range) * (term1_y2 - term2_y2)
    # Fix potential negative values due to numerical dust or logic
    pdf_y2[pdf_y2 < 0] = 0 
    pdf_y2[x >= 0] = 0
    
    # Plotting
    # Scaled PDFs
    scaled_pdf_x = 0.5 * pdf_x
    scaled_pdf_y1 = 0.5 * pdf_y1
    scaled_pdf_y2 = 0.5 * pdf_y2

    # Plotting
    plt.figure(figsize=(10, 6))
    
    # X - Straight (Full Gaussian)
    plt.fill_between(x, scaled_pdf_x, color='limegreen', alpha=0.3, label='Straight')
    plt.plot(x, scaled_pdf_x, color='darkgreen', linestyle='--', linewidth=2)
    
    # Y1 - Positive Part Shifted
    plt.fill_between(x, scaled_pdf_y1, color='dodgerblue', alpha=0.5, label='Right Turn')
    plt.plot(x, scaled_pdf_y1, color='navy', linestyle='-', linewidth=2)
    
    # Y2 - Negative Part Shifted
    plt.fill_between(x, scaled_pdf_y2, color='tomato', alpha=0.5, label='Left Turn')
    plt.plot(x, scaled_pdf_y2, color='darkred', linestyle='-', linewidth=2)
    
    plt.title(f'Distributions of Different Modes (Mixture)')
    plt.xlabel('X')
    plt.ylabel('Probability Density')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    output_file = 'distribution_plot_split-2.png'
    plt.savefig(output_file, dpi=500)
    print(f"Split plot saved to {output_file}-2")


if __name__ == "__main__":
    plot_mixture_distributions()
    plot_split_distributions()
    plot_mixture_distributions_2()
    plot_split_distributions_2()
    
