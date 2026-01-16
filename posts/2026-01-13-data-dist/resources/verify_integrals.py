
import numpy as np
from scipy.stats import norm

def calculate_integrals():
    mu_x = 0.0
    sigma_x = 0.4
    z_range = 0.3
    
    # Use a wide range and fine grid for numerical integration
    x = np.linspace(-10, 10, 10000)
    dx = x[1] - x[0]
    
    # PDF of X
    pdf_x = norm.pdf(x, mu_x, sigma_x)
    total_x = np.sum(pdf_x) * dx
    
    # PDF Y1 (mass of X>0 convolved)
    # Formula from plot_distribution.py
    term1_y1 = norm.cdf(x, mu_x, sigma_x)
    term2_y1 = norm.cdf(np.maximum(0, x - z_range), mu_x, sigma_x)
    pdf_y1 = (1.0 / z_range) * (term1_y1 - term2_y1)
    pdf_y1[x <= 0] = 0
    total_y1 = np.sum(pdf_y1) * dx
    
    # PDF Y2 (mass of X<0 convolved)
    upper_limit_y2 = np.minimum(0, x + z_range)
    term1_y2 = norm.cdf(upper_limit_y2, mu_x, sigma_x)
    term2_y2 = norm.cdf(x, mu_x, sigma_x)
    pdf_y2 = (1.0 / z_range) * (term1_y2 - term2_y2)
    pdf_y2[pdf_y2 < 0] = 0
    pdf_y2[x >= 0] = 0
    total_y2 = np.sum(pdf_y2) * dx
    
    # Scaled Integrals
    scaled_x_mass = 0.5 * total_x
    scaled_y1_mass = 0.5 * total_y1
    scaled_y2_mass = 0.5 * total_y2
    
    total_sum = scaled_x_mass + scaled_y1_mass + scaled_y2_mass
    
    print(f"Original X Mass: {total_x:.4f}")
    print(f"Original Y1 Mass (Convolution of X>0): {total_y1:.4f}")
    print(f"Original Y2 Mass (Convolution of X<0): {total_y2:.4f}")
    print("-" * 30)
    print(f"Scaled X Mass (0.5 * X): {scaled_x_mass:.4f}")
    print(f"Scaled Y1 Mass (0.25 * Y1): {scaled_y1_mass:.4f}")
    print(f"Scaled Y2 Mass (0.25 * Y2): {scaled_y2_mass:.4f}")
    print("-" * 30)
    print(f"Total Sum: {total_sum:.4f}")

if __name__ == "__main__":
    calculate_integrals()
