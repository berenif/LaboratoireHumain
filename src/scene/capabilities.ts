export interface GraphicsCapabilities {
  readonly webgl2: boolean;
  readonly canvas2d: boolean;
  readonly webglError?: string;
  readonly canvas2dError?: string;
  readonly vendor?: string;
  readonly renderer?: string;
}

function messageFrom(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

/** Uses separate temporary canvases so failure of one API cannot poison the other. */
export function checkGraphicsCapabilities(): GraphicsCapabilities {
  if (typeof document === "undefined") {
    const error = "Graphics capability checks require a browser document.";
    return {
      webgl2: false,
      canvas2d: false,
      webglError: error,
      canvas2dError: error,
    };
  }

  let canvas2d = false;
  let canvas2dError: string | undefined;
  try {
    const canvas = document.createElement("canvas");
    canvas.width = 2;
    canvas.height = 2;
    const context = canvas.getContext("2d");
    if (!context) {
      canvas2dError = "CanvasRenderingContext2D creation returned null.";
    } else {
      context.fillStyle = "#000";
      context.fillRect(0, 0, 1, 1);
      canvas2d = true;
    }
  } catch (error) {
    canvas2dError = messageFrom(error);
  }

  let webgl2 = false;
  let webglError: string | undefined;
  let vendor: string | undefined;
  let renderer: string | undefined;
  try {
    const canvas = document.createElement("canvas");
    canvas.width = 2;
    canvas.height = 2;
    const context = canvas.getContext("webgl2", {
      alpha: false,
      antialias: false,
      depth: true,
      stencil: false,
    });
    if (!context) {
      webglError = "WebGL2RenderingContext creation returned null.";
    } else {
      const debug = context.getExtension("WEBGL_debug_renderer_info");
      vendor = debug
        ? String(context.getParameter(debug.UNMASKED_VENDOR_WEBGL))
        : String(context.getParameter(context.VENDOR));
      renderer = debug
        ? String(context.getParameter(debug.UNMASKED_RENDERER_WEBGL))
        : String(context.getParameter(context.RENDERER));
      webgl2 = true;
    }
  } catch (error) {
    webglError = messageFrom(error);
  }

  return {
    webgl2,
    canvas2d,
    ...(webglError ? { webglError } : {}),
    ...(canvas2dError ? { canvas2dError } : {}),
    ...(vendor ? { vendor } : {}),
    ...(renderer ? { renderer } : {}),
  };
}
