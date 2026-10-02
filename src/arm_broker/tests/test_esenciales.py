"""Pruebas unitarias para validar las funciones esenciales de la cinemática directa (FK)."""

import math
import unittest
from arm_broker import fk


class TestItem1FK(unittest.TestCase):

    def test_fk_pose_cero(self):
        """Con q = 0 el brazo apunta hacia arriba: x = d6 y y = -d4."""
        x, y, z = fk.fk([0.0] * 6)
        self.assertAlmostEqual(x, 45.6, places=1)
        self.assertAlmostEqual(y, -63.4, places=1)
        self.assertAlmostEqual(z, 412.7, places=1)

    def test_predicciones_del_diseno_previo(self):
        """Las predicciones congeladas en docs/tabla_dh.md salen del código actual."""
        esperado = {
            'ready': ([0, -0.5, 0.5, 0, 0.5, 0], (92.9, -41.5, 399.2)),
            'baja': ([0, -1.2, 1.2, 0, 0, 0], (148.5, -63.4, 342.3)),
        }
        for nombre, (q, xyz) in esperado.items():
            for a, b in zip(fk.fk(q), xyz):
                self.assertAlmostEqual(a, b, places=1, msg=nombre)

    def test_limites_articulares(self):
        """Valida que la función detecte configuraciones dentro y fuera de rango."""
        ok, _ = fk.dentro_de_limites([0.0] * 6)
        self.assertTrue(ok)

        fuera, _ = fk.dentro_de_limites([3.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertFalse(fuera)

    def test_workspace(self):
        """Valida las restricciones del espacio de trabajo geométrico."""
        ok, _ = fk.dentro_del_workspace([0.0] * 6)
        self.assertTrue(ok)


if __name__ == '__main__':
    unittest.main()
