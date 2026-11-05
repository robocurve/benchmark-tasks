"""Check SDK credentials/model access only; never warm, reset or request an action."""
import argparse
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--model', default='dreamlabs/dream-1-beta')
    args = p.parse_args()
    result = {'model': args.model, 'inference_requested': False, 'ready': False}
    client = None
    try:
        import dream
        from dream import credentials
        c = credentials.load()
        result['key_present'] = bool(c.api_key)
        result['official_endpoints'] = (
            c.gateway_url.rstrip('/') == credentials.DEFAULT_GATEWAY_URL
            and c.platform_url.rstrip('/') == credentials.DEFAULT_PLATFORM_URL
            and c.data_url.rstrip('/') == credentials.DEFAULT_DATA_URL)
        if not result['key_present']:
            result['reason'] = 'missing_credentials'
        elif not result['official_endpoints']:
            result['reason'] = 'nondefault_endpoints_require_review'
        else:
            client = dream.DreamClient()
            result['available_models'] = [m.id for m in client.get_models()]
            result['ready'] = args.model in result['available_models']
            if not result['ready']:
                result['reason'] = 'pinned_model_not_available'
    except Exception as error:
        # Do not persist arbitrary server messages, credential objects or environment.
        result['error_type'] = type(error).__name__
    finally:
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['ready'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
