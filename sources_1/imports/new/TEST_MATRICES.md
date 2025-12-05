# Mapa de Memoria - Matrices con Valores Flotantes Complejos

## Distribución de Memoria

```
Dirección  | Word | Contenido
-----------|------|--------------------------------------------
0x00-0x3C  | 0-15 | Memoria general (disponible)
0x40       | 16   | A[0][0] = 1.5  (0x3FC00000)
0x44       | 17   | A[0][1] = 2.3  (0x40133333)
0x48       | 18   | A[1][0] = 3.7  (0x406CCCCD)
0x4C       | 19   | A[1][1] = 4.2  (0x40866666)
0x50       | 20   | B[0][0] = 5.1  (0x40A33333)
0x54       | 21   | B[0][1] = 6.8  (0x40D9999A)
0x58       | 22   | B[1][0] = 7.4  (0x40ECCCCD)
0x5C       | 23   | B[1][1] = 8.9  (0x410E6666)
0x60       | 24   | C[0][0] = 0.0  (espacio para resultado)
0x64       | 25   | C[0][1] = 0.0  (espacio para resultado)
0x68       | 26   | C[1][0] = 0.0  (espacio para resultado)
0x6C       | 27   | C[1][1] = 0.0  (espacio para resultado)
0x70-0xFC  | 28-63| Memoria general (disponible)
```

## Matrices de Entrada (Pre-cargadas en dmem)

### Matriz A (base 0x40)
```
A = [[1.5, 2.3],
     [3.7, 4.2]]
```

**Conversión IEEE-754:**
| Elemento | Decimal | Hexadecimal | Binario (IEEE-754) |
|----------|---------|-------------|-------------------|
| A[0][0] | 1.5 | 0x3FC00000 | 0 01111111 10000000000000000000000 |
| A[0][1] | 2.3 | 0x40133333 | 0 10000000 00100110011001100110011 |
| A[1][0] | 3.7 | 0x406CCCCD | 0 10000000 11011001100110011001101 |
| A[1][1] | 4.2 | 0x40866666 | 0 10000001 00001100110011001100110 |

### Matriz B (base 0x50)
```
B = [[5.1, 6.8],
     [7.4, 8.9]]
```

**Conversión IEEE-754:**
| Elemento | Decimal | Hexadecimal | Binario (IEEE-754) |
|----------|---------|-------------|-------------------|
| B[0][0] | 5.1 | 0x40A33333 | 0 10000001 01000110011001100110011 |
| B[0][1] | 6.8 | 0x40D9999A | 0 10000001 10110011001100110011010 |
| B[1][0] | 7.4 | 0x40ECCCCD | 0 10000001 11011001100110011001101 |
| B[1][1] | 8.9 | 0x410E6666 | 0 10000010 00011100110011001100110 |

## Cálculo del Resultado Esperado

### Matriz C = A × B (base 0x60)

**Cálculo manual paso por paso:**

```
C[0][0] = A[0][0]*B[0][0] + A[0][1]*B[1][0]
        = 1.5 × 5.1 + 2.3 × 7.4
        = 7.65 + 17.02
        = 24.67

C[0][1] = A[0][0]*B[0][1] + A[0][1]*B[1][1]
        = 1.5 × 6.8 + 2.3 × 8.9
        = 10.2 + 20.47
        = 30.67

C[1][0] = A[1][0]*B[0][0] + A[1][1]*B[1][0]
        = 3.7 × 5.1 + 4.2 × 7.4
        = 18.87 + 31.08
        = 49.95

C[1][1] = A[1][0]*B[0][1] + A[1][1]*B[1][1]
        = 3.7 × 6.8 + 4.2 × 8.9
        = 25.16 + 37.38
        = 62.54
```

**Matriz resultado:**
```
C = [[24.67, 30.67],
     [49.95, 62.54]]
```

**Conversión IEEE-754 del resultado:**
| Elemento | Decimal | Hexadecimal | 
|----------|---------|-------------|
| C[0][0] | 24.67 | 0x41C55C29 |
| C[0][1] | 30.67 | 0x41F55C29 |
| C[1][0] | 49.95 | 0x4247CCCD |
| C[1][1] | 62.54 | 0x427A2E8C |

## Registros Después de la Ejecución

| Registro | Dirección/Valor | Descripción |
|----------|-----------------|-------------|
| x10 | 0x00000040 | Dirección base de A |
| x11 | 0x00000050 | Dirección base de B |
| x12 | 0x00000060 | Dirección base de C |
| x13 | TBD | Flag/resultado de MATMUL2 |
| x15 | 0x41C55C29 | C[0][0] = 24.67 |
| x16 | 0x41F55C29 | C[0][1] = 30.67 |
| x17 | 0x4247CCCD | C[1][0] = 49.95 |
| x18 | 0x427A2E8C | C[1][1] = 62.54 |

## Verificación en Python

```python
import struct

# Función para convertir float a hex IEEE-754
def float_to_hex(f):
    return hex(struct.unpack('>I', struct.pack('>f', f))[0])

# Función para convertir hex IEEE-754 a float
def hex_to_float(h):
    return struct.unpack('>f', struct.pack('>I', int(h, 16)))[0]

# Matriz A
A = [[1.5, 2.3], [3.7, 4.2]]
# Matriz B
B = [[5.1, 6.8], [7.4, 8.9]]

# Calcular C = A × B
C = [[0, 0], [0, 0]]
for i in range(2):
    for j in range(2):
        for k in range(2):
            C[i][j] += A[i][k] * B[k][j]

print("Resultado C:")
for i in range(2):
    for j in range(2):
        print(f"C[{i}][{j}] = {C[i][j]:.2f} → {float_to_hex(C[i][j])}")
```

**Salida esperada:**
```
C[0][0] = 24.67 → 0x41c55c29
C[0][1] = 30.67 → 0x41f55c29
C[1][0] = 49.95 → 0x4247cccd
C[1][1] = 62.54 → 0x427a2e8c
```

## Notas de Implementación

- **Valores complejos**: Todos los valores tienen decimales no nulos (1.5, 2.3, 3.7, etc.)
- **Precisión IEEE-754**: Los valores están en formato single-precision (32 bits)
- **Redondeo**: Los valores hexadecimales muestran el redondeo inherente de IEEE-754
- **Verificación**: Los resultados pueden variar ligeramente debido al redondeo de la FPU

## Verificación en Testbench

Agregar al testbench para verificar matrices cargadas:

```systemverilog
initial begin
  #50; // Esperar después del reset
  
  $display("\n========== VERIFICACIÓN DE MATRICES ==========");
  $display("Matriz A:");
  $display("  A[0][0] = %h (1.5)", dut.dmem.RAM[16]);
  $display("  A[0][1] = %h (2.3)", dut.dmem.RAM[17]);
  $display("  A[1][0] = %h (3.7)", dut.dmem.RAM[18]);
  $display("  A[1][1] = %h (4.2)", dut.dmem.RAM[19]);
  
  $display("Matriz B:");
  $display("  B[0][0] = %h (5.1)", dut.dmem.RAM[20]);
  $display("  B[0][1] = %h (6.8)", dut.dmem.RAM[21]);
  $display("  B[1][0] = %h (7.4)", dut.dmem.RAM[22]);
  $display("  B[1][1] = %h (8.9)", dut.dmem.RAM[23]);
  $display("==============================================\n");
end
```
